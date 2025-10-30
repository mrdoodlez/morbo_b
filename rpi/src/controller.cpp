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

class GlobalState
{
public:
    struct VodomState
    {
        uint64_t t_us = 0;
        bool valid = false;
        float dx_f = 0.f; // filtered
        float dy_f = 0.f; // filtered
        float quality = 0.f;
    };

    struct ControlTargets
    {
        float dx_target_m; // where we want the tag relative to us
        float dy_target_m;
    };

    void setVodom(const VodomMsg &v)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        // 1) basic validity
        bool detected = (v.status != VodomMsg::VODOM_NOT_DETECTED);
        if (!detected)
        {
            vodom_.valid = false;
            vodom_.t_us = v.t_us;
            return;
        }

        // 2) quality gate (tune!)
        if (v.quality < 0.15f)
        {
            // too weak → mark invalid but keep timestamp
            vodom_.valid = false;
            vodom_.t_us = v.t_us;
            return;
        }

        // 3) EMA filter
        // alpha closer to 1.0 → more responsive
        constexpr float alpha = 0.4f;
        if (!vodom_.valid)
        {
            // first detection → hard set
            vodom_.dx_f = v.dx_m;
            vodom_.dy_f = v.dy_m;
        }
        else
        {
            vodom_.dx_f = (1.f - alpha) * vodom_.dx_f + alpha * v.dx_m;
            vodom_.dy_f = (1.f - alpha) * vodom_.dy_f + alpha * v.dy_m;
        }

        vodom_.t_us = v.t_us;
        vodom_.quality = v.quality;
        vodom_.valid = true;
    }

    VodomState getVodom() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return vodom_;
    }

    void setTarget(float dx, float dy)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        targets_.dx_target_m = dx;
        targets_.dy_target_m = dy;
    }

    ControlTargets getTargets() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return targets_;
    }

private:
    mutable std::mutex mtx_;
    VodomState vodom_{};
    ControlTargets targets_{};
} g_state;

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
    Controller_LoadParams(); // TODO: move this to main()

    g_state.setTarget(1.0, 0.0); // <- read from params;

    Comm_Start();

    int rc = _RoverConfig();
    if (rc)
    {
        vlog.text << "Rover config failed, rc: " << rc << std::endl;
        return -10;
    }

    rc = Vodom_Start(params.videoDev /* this should be not arg as well */);
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

static void _SendRelativeMove(float dx_m, float dy_m)
{
    HIP_Payload_SetPos_t pd;

    pd.x = dx_m;
    pd.y = dy_m;
    pd.flags = HIP_SetPos_Flags_IsRelative;

    HostIface_PutData(HIP_MSG_SET_POS, (uint8_t *)&pd, sizeof(pd));
    HostIface_Send();
}

// how old VO can be
static constexpr uint64_t VODOM_STALE_US = 300000; // 300 ms
// command rate
static constexpr std::chrono::milliseconds CMD_PERIOD_MS(1000); // 1 Hz

static void _OnHeartbeat(uint64_t ts_ms)
{
    static auto last_cmd_time = std::chrono::steady_clock::now();

    GlobalState::VodomState vo = g_state.getVodom();
    GlobalState::ControlTargets tgt = g_state.getTargets();

    uint64_t now_us = ts_ms * 1000ULL;
    bool fresh = vo.valid && (now_us - vo.t_us) < VODOM_STALE_US;

    if (!fresh)
    {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (now - last_cmd_time < CMD_PERIOD_MS)
    {
        return;
    }
    last_cmd_time = now;

    float ex = tgt.dx_target_m - vo.dx_f; // forward/back
    float ey = tgt.dy_target_m - vo.dy_f; // left/right

    float step_x = ex;
    float step_y = ey;

    // limit step size per command
    constexpr float STEP_MAX = 0.25f; // TODO: <- this is a parameter as well
    auto clamp_step = [](float v)
    {
        if (v > STEP_MAX)
            return STEP_MAX;
        if (v < -STEP_MAX)
            return -STEP_MAX;
        return v;
    };
    step_x = clamp_step(step_x);
    step_y = clamp_step(step_y);

    _SendRelativeMove(step_x, step_y);

    vlog.text << "[CTRL] send REL_MOVE dx=" << step_x
              << " dy=" << step_y
              << " (err x=" << ex << " y=" << ey << ")"
              << std::endl;
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

    const char *color =
        (v.status == VodomMsg::VodomStatus::VODOM_DETECTED_RED)
            ? "RED"
        : (v.status == VodomMsg::VodomStatus::VODOM_DETECTED_GREEN)
            ? "GREEN"
            : "UNKNOWN";

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

    g_state.setVodom(v);
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