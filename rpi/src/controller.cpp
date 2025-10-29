#include "controller.h"
#include "params.h"
#include "comm.h"
#include "vodom.h"
#include "host_interface_cmds.h"
#include "host_interface.h"
#include "logger.h"

#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstring>
#include <iomanip>

static int _SetWorkMode(uint8_t iMode, uint8_t fcMode);
static int _EnableMessage(uint16_t msgId, uint16_t periodMs);

static void _HandleMessage(const ControllerMsg &m);
static void _OnHeartbeat(uint64_t ts_ms);

static int _RoverConfig();

static void _Worker();

extern std::atomic<bool> g_stop;

enum IMU_Mode_t
{
    IMU_Mode_Idle,    /*0*/
    IMU_Mode_CalAcc,  /*1*/
    IMU_Mode_CalGyro, /*2*/
    IMU_Mode_CalMag,  /*3*/
    IMU_Mode_Fusion,  /*4*/
};

enum FlightScenario_t
{
    FlightScenario_None,
    FlightScenario_Debug,
    FlightScenario_VelSet,
    FlightScenario_GoTo,

    FlightScenario_Total,
};

static std::thread g_ctrlThread;
static std::mutex g_mtx;
static std::condition_variable g_cv;
static std::queue<ControllerMsg> g_q;

/*******************************************************************************/

uint64_t Controller_NowMs()
{
    using clock = std::chrono::steady_clock;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               clock::now().time_since_epoch())
        .count();
}

int Controller_Start(const ControllerParams &params)
{
    Controller_LoadParams();

    Comm_Start();

    int rc = _RoverConfig();
    if (rc)
    {
        vlog.text << "Rover config failed, rc: " << rc << std::endl;
        return -10;
    }

    rc = Vodom_Start(params.videoDev);
    if (rc)
    {
        vlog.text << "Visual odometry start failed, rc: " << rc << std::endl;
        return -10;
    }

    g_ctrlThread = std::thread(_Worker);
    return 0;
}

static int _RoverConfig()
{
    if (_SetWorkMode(IMU_Mode_Fusion, FlightScenario_GoTo))
        return -10;

    if (_EnableMessage(HIP_MSG_PVT, 100))
        return -20;

    if (_EnableMessage(HIP_MSG_WHT, 100))
        return -30;

    return 0;
}

/*******************************************************************************/

static int _SendCommand(uint16_t id, const uint8_t *buff, size_t len)
{
    HostIface_PutData(id, buff, len);
    HostIface_Send();

    bool isAck;
    int rc = WaitForAck(id, std::chrono::milliseconds(500), isAck);

    if ((rc == 0) && isAck)
        return 0;

    vlog.text << "command " << id << " not acked" << std::endl;

    return -10;
}

static int _SetWorkMode(uint8_t iMode, uint8_t fcMode)
{
    HIP_Payload_WM_t wm;
    wm.fcMode = fcMode;
    wm.imuMode = iMode;

    return _SendCommand(HIP_MSG_WM, (uint8_t *)&wm, sizeof(wm));
}

static int _EnableMessage(uint16_t msgId, uint16_t periodMs)
{
    HIP_Payload_EM_t em;

    em.msgId = msgId;
    em.msgPeriod = periodMs / 100;

    return _SendCommand(HIP_MSG_EM, (uint8_t *)&em, sizeof(em));
}

/*******************************************************************************/

static constexpr int HEARTBEAT_HZ = 5;
static constexpr std::chrono::milliseconds HEARTBEAT_PERIOD_MS{1000 / HEARTBEAT_HZ};

static void _Worker()
{
    using clock = std::chrono::steady_clock;

    auto next_tick = clock::now() + HEARTBEAT_PERIOD_MS;

    while (!g_stop.load(std::memory_order_relaxed))
    {
        std::unique_lock<std::mutex> lk(g_mtx);
        // Wake on either: new message OR heartbeat deadline
        g_cv.wait_until(lk, next_tick, []
                        { return !g_q.empty() || g_stop.load(std::memory_order_relaxed); });

        if (g_stop.load(std::memory_order_relaxed))
            break;

        // Drain all pending messages (avoid backlog)
        while (!g_q.empty())
        {
            ControllerMsg m = g_q.front();
            g_q.pop();
            lk.unlock();
            _HandleMessage(m);
            lk.lock();
        }
        lk.unlock();

        // If it's time (or we missed a beat), run heartbeat
        auto now = clock::now();
        if (now >= next_tick)
        {
            _OnHeartbeat(Controller_NowMs());

            // advance next_tick (catch up if we fell behind)
            do
            {
                next_tick += HEARTBEAT_PERIOD_MS;
            } while (next_tick <= now);
        }
    }
}

/*******************************************************************************/

int Controller_PostMessage(const ControllerMsg &m)
{
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_q.push(m);
    }
    g_cv.notify_one();
    return 0;
}

static void _OnHeartbeat(uint64_t ts_ms)
{
    // Periodic control tick (plan, fuse, emit commands, timeouts, etc.)
    // Example skeleton:
    // controller.FuseSensors();
    // controller.EvaluateScenario();
    // controller.EmitActuatorCommands();
    (void)ts_ms;

    vlog.text << __func__ << std::endl;
}

static void _HandlePVT(const HIP_Payload_PVT_t &pvt)
{
    vlog.text << std::fixed << std::setprecision(3)
              << "[PVT] pos=(" << pvt.position[0] << ", "
              << pvt.position[1] << ", "
              << pvt.position[2] << ") "
              << "vel=(" << pvt.velocity[0] << ", "
              << pvt.velocity[1] << ", "
              << pvt.velocity[2] << ") "
              << "t=" << pvt.time
              << std::endl;
}

static void _HandleWHT(const HIP_Payload_WHT_t &wht)
{
    vlog.text << std::fixed << std::setprecision(3)
              << "[WHT] heading=" << wht.heading
              << " ang_vel=" << wht.ang_velocity
              << " t=" << wht.time
              << std::endl;
}

// Helper: extract typed payload safely
template <typename T>
static void _ExtractPayload(const HIP_Cmd_t &rov, T &out)
{
    std::memcpy(&out, &rov.payload, sizeof(T));
}

static void _HandleRover(const HIP_Cmd_t &rov)
{
    // (Optional) CRC check here; return early on failure

    switch (rov.header.cmd)
    {
    case HIP_MSG_PVT:
    {
        HIP_Payload_PVT_t p{};
        _ExtractPayload(rov, p);
        _HandlePVT(p);
        break;
    }
    case HIP_MSG_WHT:
    {
        HIP_Payload_WHT_t w{};
        _ExtractPayload(rov, w);
        _HandleWHT(w);
        break;
    }
    default:
        vlog.text << "unknown rover msg: " << rov.header.cmd << std::endl;
        break;
    }
}

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

static void _HandleVodom(const VodomMsg &v)
{
    using std::fixed;
    using std::setprecision;

    if (v.status == VodomMsg::VodomStatus::VODOM_NOT_DETECTED)
    {
        vlog.text << "[VODOM] not detected, t_us=" << v.t_us << std::endl;
        return;
    }

    const char *color =
        (v.status == VodomMsg::VodomStatus::VODOM_DETECTED_RED)
            ? "RED" : (v.status == VodomMsg::VodomStatus::VODOM_DETECTED_GREEN)
            ? "GREEN" : "UNKNOWN";

    vlog.text << fixed << setprecision(3)
              << "[VODOM] Detected " << color
              << " dx=" << v.dx_m << " m"
              << " dy=" << v.dy_m << " m"
              << " range=" << v.range_m << " m"
              << " bearing=" << setprecision(1) << rad2deg(v.bearing_rad) << " deg"
              << setprecision(3)
              << " d_px=" << v.d_px
              << " q=" << v.quality
              << " t_us=" << v.t_us
              << std::endl;
}

static void _HandleMessage(const ControllerMsg &m)
{
    switch (m.type)
    {
    case ControllerMsg::Type::TYPE_ROVER:
        _HandleRover(m.payload.rovData);
        break;
    case ControllerMsg::Type::TYPE_VODOM:
        _HandleVodom(m.payload.vodom);
        break;
    default:
        break;
    }
}