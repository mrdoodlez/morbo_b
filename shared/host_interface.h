#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include <stdint.h>
#include "host_interface_cmds.h"

#ifdef __cplusplus
extern "C"
{
#endif

int HostIface_PutData(uint16_t id, const uint8_t *buff, uint16_t len);
int HostIface_Send();

void HostIface_Listen();

void Controller_NewCommand(const HIP_Cmd_t* cmd);

#ifdef __cplusplus
}
#endif

#endif // _HOST_INTERFACE_H_