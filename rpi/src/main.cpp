#include <iostream>
#include "controller.h"
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include "logger.h"

std::atomic<bool> g_stop{false};

extern "C" void on_signal(int)
{
    g_stop.store(true, std::memory_order_relaxed);
}

int main()
{
    ::signal(SIGINT, on_signal);
    ::signal(SIGTERM, on_signal);
    ::signal(SIGPIPE, SIG_IGN);

    std::cout << "vpos starts\n";
    int rc = 0;

    ControllerParams params;
    params.mcuDev = "/dev/ttyUSB0";
    params.videoDev = "/dev/video0";
    if ((rc = Controller_Start(params)) != 0)
    {
        vlog.text << "controller start error: " << rc << std::endl;
        return -20;
    }

    while (!g_stop.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    vlog.CloseAll();

    std::cout << "vlog stops" << std::endl;

    return 0;
}
