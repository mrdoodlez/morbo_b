#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "main.h"
#include "motion_di.h"
#include "host_interface_cmds.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	MachineState_Disarmed,
	MachineState_Armed,
	MachineState_Debug,
	MachineState_HardFault,
} MachineState_t;

void Controller_Task();

void Controller_NewMeas(const MDI_output_t *mdiData);

void Controller_NewCommand(const HIP_Cmd_t* cmd);

void Controller_HandleFatal();

MachineState_t GetMachineState();

#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
