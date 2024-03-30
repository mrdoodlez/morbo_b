#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include "main.h"

int HostIface_Start();
int HostIface_SendData(uint16_t id, const uint8_t *buff, uint16_t len);

#endif // _HOST_INTERFACE_H_