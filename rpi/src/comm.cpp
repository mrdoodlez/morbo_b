#include <thread>
#include "../../shared/host_interface.h"

void Comm_Start()
{
    std::thread([] {
        HostIface_Listen();
    }).detach();
}