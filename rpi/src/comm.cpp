#include <thread>
#include "../../shared/host_interface.h"

void Comm_Start()
{
    std::thread([] {
        HostIface_Listen();
    }).detach();
}

void Controller_NewCommand(const HIP_Cmd_t* cmd)
{

}