#include <thread>
#include "host_interface_cmds.h"
#include "host_interface.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <map>

static void _Pinger();
static void _Pinger_Start();

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

std::map<uint16_t, HIP_CmdHandler> g_handlTable;

void Comm_Start()
{
    g_handlTable[HIP_MSG_PING] = _OnPong;
    g_handlTable[HIP_MSG_ACK ] = _OnAck;
    g_handlTable[HIP_MSG_NAK ] = _OnNack;

    std::thread([] {
        HostIface_Listen();
    }).detach();

    _Pinger_Start();
}

void _Pinger_Start()
{
    std::thread([] {
        _Pinger();
    }).detach();
}

void Controller_NewCommand(const HIP_Cmd_t* cmd)
{
    HIP_CmdHandler handler = nullptr;

    //std::cout << __func__ << ' ' << cmd->header.cmd << '\n';

    auto it = g_handlTable.find(cmd->header.cmd);
    if (it != g_handlTable.end())
        handler = it->second;

    if (handler)
        handler(cmd);
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

static void _Pinger()
{
    uint16_t txSeq = 0;
    for (;;)
    {
        std::cout << "ping seq: " << txSeq << '\n';

        HostIface_PutData(HIP_MSG_PING, (uint8_t *)&txSeq, sizeof(txSeq));
        HostIface_Send();

        if (_WaitForPong(txSeq, std::chrono::milliseconds(200)) != 0)
        {
            // TODO: handle error
        }
        else
        {
            std::cout << "pong\n";
        }

        txSeq++;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}