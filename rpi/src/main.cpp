#include <iostream>
#include "controller.h"
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include "logger.h"
#include "params.h"

std::atomic<bool> g_stop{false};

extern "C" void on_signal(int)
{
    g_stop.store(true, std::memory_order_relaxed);
}

int main(int argc, char **argv)
{
    ::signal(SIGINT, on_signal);
    ::signal(SIGTERM, on_signal);
    ::signal(SIGPIPE, SIG_IGN);

    std::string params;

    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "--params")
        {
            params = argv[i + 1];
            break;
        }
    }

    if (params.empty())
    {
        std::cerr << "ERROR: No params file provided.\n"
                  << "Usage: ./vpos --params config.json\n";
        return -10;
    }

    std::cout << "vpos starts\n";
    int rc = 0;

    if ((rc = Controller_LoadParams(params)) != 0)
    {
        vlog.text << "params load error: " << rc << std::endl;
        return -20;
    }

    if ((rc = Controller_Start()) != 0)
    {
        vlog.text << "controller start error: " << rc << std::endl;
        return -30;
    }

    while (!g_stop.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    vlog.CloseAll();

    std::cout << "vlog stops" << std::endl;

    return 0;
}
