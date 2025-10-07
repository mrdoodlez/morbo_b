#pragma once

#include "host_interface_cmds.h"

int Controller_Start();

#ifdef __cplusplus
extern "C"
{
#endif

void Controller_NewCommand(const HIP_Cmd_t* cmd);

#ifdef __cplusplus
}
#endif