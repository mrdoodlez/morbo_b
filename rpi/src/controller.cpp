#include "controller.h"
#include "comm.h"
#include "host_interface_cmds.h"
#include "host_interface.h"
#include <thread>
#include <chrono>
#include <condition_variable>
#include <mutex>

static void Pinger();
static void _Pinger_Start();

int Controller_Start()
{
    Comm_Start();

    return 0;
}

void Controller_NewCommand(const HIP_Cmd_t* cmd)
{

}

/*******************************************************************************
 *
 *  Pinger tracker code here
 */ 

std::mutex              g_mtx;
std::condition_variable g_cv;

uint16_t g_lastSeq = 0;
uint64_t g_seenVer = 0;

static void _OnPong(uint16_t seq)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_lastSeq = seq;
    ++g_seenVer;
    g_cv.notify_all();
}

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

static void _Pinger_Start()
{
    std::thread([] {
        _Pinger_Start();
    }).detach();
}

static void _Pinger()
{
    uint16_t txSeq = 0;
    for (;;)
    {
        HostIface_PutData(HIP_MSG_PING, (uint8_t *)&txSeq, sizeof(txSeq));
        HostIface_Send();

        if (!_WaitForPong(txSeq, std::chrono::milliseconds(200)))
        {
            // TODO: handle error
        }

        txSeq++;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

/******************************************************************************/