#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "main.h"
#include "motion_di.h"
#include "host_interface_cmds.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Controller_Task();

void Controller_NewMeas(const MDI_output_t *mdiData);

void Controller_NewCommand(const HIP_Cmd_t* cmd);

#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
