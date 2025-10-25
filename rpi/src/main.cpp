#include <iostream>
#include "serial.h"
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
    if ((rc = Serial_Init("/dev/ttyUSB0")) != 0)
    {
        std::cout << "serial open error: " << rc << std::endl;
        return -10;
    }

    ControllerParams params;
    params.videoDev = "/dev/video0";
    if ((rc = Controller_Start(params)) != 0)
    {
        std::cout << "controller start error: " << rc << std::endl;
        return -20;
    }

    while (!g_stop.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    vlog.CloseAll();

    return 0;
}
