#include <thread>
#include "host_interface_cmds.h"
#include "host_interface.h"
#include "controller.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <map>
#include "serial.h"
#include "logger.h"
#include "tcp_server.h"

static void _Pinger(int comm);
static void _Pinger_Start(int comm);

std::mutex              g_mtx;
std::condition_variable g_cv;

// --- PONG state ---
static uint16_t g_lastSeq = 0;

// --- ACK/NACK state ---
static uint16_t g_lastAckCmdId = 0;
static bool     g_lastAckIsAck = false;

static uint64_t g_seenVer = 0;

using HIP_CmdHandler = void(*)(const HIP_Cmd_t*);

// Command handlers
static void _OnPong(const HIP_Cmd_t*);
static void _OnAck (const HIP_Cmd_t*);
static void _OnNack(const HIP_Cmd_t*);

std::map<uint16_t, HIP_CmdHandler> g_mcuHandlers;

/*******************************************************************************/

extern "C" void _Comm_NewMessage(int, const HIP_Cmd_t* cmd)
{
    HIP_CmdHandler handler = nullptr;

    // vlog.text << __func__ << ' ' << cmd->header.cmd << std::endl;

    auto it = g_mcuHandlers.find(cmd->header.cmd);
    if (it != g_mcuHandlers.end())
        handler = it->second;

    if (handler)
    {
        handler(cmd);
    }
    else
    {
        ControllerMsg rovMsg;
        rovMsg.ts_ms = Controller_NowMs();
        rovMsg.type = ControllerMsg::Type::TYPE_ROVER;
        rovMsg.payload.rovData = *cmd;

        Controller_PostMessage(rovMsg);
    }
}

extern "C" void _Host_NewMessage(int port, const HIP_Cmd_t* cmd)
{
    if (cmd->header.cmd == HIP_MSG_PING)
    {
        uint16_t rxSeq = ((HIP_Ping_t*)cmd)->payload.seqNumber;
        HostIface_PutData(port, HIP_MSG_PING, (uint8_t *)&rxSeq, sizeof(rxSeq));
        HostIface_Send(port);
    }
    else
    {
        ControllerMsg hostMsg;
        hostMsg.ts_ms = Controller_NowMs();
        hostMsg.type = ControllerMsg::Type::TYPE_HOST;
        hostMsg.payload.rovData = *cmd;

        Controller_PostMessage(hostMsg);
    }
}

/*******************************************************************************/

void Comm_Start(int mcu, int hst)
{
    HostIface_Callbacks_t mcuCbs =
    {
        .read_fn = Serial_Read,
        .write_fn = Serial_Write,
        .handler = _Comm_NewMessage,
    };

    g_mcuHandlers[HIP_MSG_PING] = _OnPong;
    g_mcuHandlers[HIP_MSG_ACK ] = _OnAck;
    g_mcuHandlers[HIP_MSG_NAK ] = _OnNack;

    HostIface_Register(mcu, &mcuCbs);

    std::thread([mcu] {
        HostIface_Listen(mcu);
    }).detach();

    _Pinger_Start(mcu);

    HostIface_Callbacks_t hstCbs =
    {
        .read_fn = Serial_Read, //P2pLink_Read,
        .write_fn = Serial_Write, //P2pLink_Write,
        .handler = _Host_NewMessage,
    };

    HostIface_Register(hst, &hstCbs);

    std::thread([hst] {
        HostIface_Listen(hst);
    }).detach();
}

void _Pinger_Start(int comm)
{
    std::thread([comm] {
        _Pinger(comm);
    }).detach();
}

/*******************************************************************************/

static void _OnPong(const HIP_Cmd_t* cmd)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_lastSeq = ((HIP_Ping_t*)cmd)->payload.seqNumber;
    ++g_seenVer;
    g_cv.notify_all();
}

static void _OnAck (const HIP_Cmd_t* cmd)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_lastAckCmdId = ((HIP_AckNak_t*)cmd)->payload.cmd;
    g_lastAckIsAck = true;
    ++g_seenVer;
    g_cv.notify_all();
}

static void _OnNack(const HIP_Cmd_t* cmd)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_lastAckCmdId = ((HIP_AckNak_t*)cmd)->payload.cmd;
    g_lastAckIsAck = false;
    ++g_seenVer;
    g_cv.notify_all();
}

int WaitForAck(uint16_t cmdIdWaitingForAck, std::chrono::milliseconds timeout, bool& isAck)
{
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + timeout;

    std::unique_lock<std::mutex> lk(g_mtx);
    uint64_t startVer = g_seenVer;

    while (steady_clock::now() < deadline)
    {
        g_cv.wait_until(lk, deadline, [&]{ return g_seenVer != startVer; });

        if (steady_clock::now() >= deadline)
            break;

        if (g_lastAckCmdId == cmdIdWaitingForAck)
        {
            isAck = g_lastAckIsAck;   // pass result to caller
            return 0;
        }

        startVer = g_seenVer;
    }

    return -1;
}

/*******************************************************************************/

static int _WaitForPong(uint16_t seq, std::chrono::milliseconds timeout)
{
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + timeout;

    std::unique_lock<std::mutex> lk(g_mtx);

    uint64_t startVer = g_seenVer;
    while (steady_clock::now() < deadline)
    {
        g_cv.wait_until(lk, deadline, [&] { return g_seenVer != startVer; });

        if (steady_clock::now() >= deadline)
            break;

        if (g_lastSeq == seq)
            return 0;

        startVer = g_seenVer;
    }

    return -1;
}

static void _Pinger(int comm)
{
    uint16_t txSeq = 0;
    int noPongCnt = 0;
    for (;;)
    {
        HostIface_PutData(comm, HIP_MSG_PING, (uint8_t *)&txSeq, sizeof(txSeq));
        HostIface_Send(comm);

        if (_WaitForPong(txSeq, std::chrono::milliseconds(200)) == 0)
        {
            noPongCnt = 0;
        }
        else if (++noPongCnt > 5)
        {
            vlog.text << "no connect with rover. exit." << std::endl;
            extern std::atomic<bool> g_stop;
            g_stop.store(true, std::memory_order_relaxed);
        }

        txSeq++;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}