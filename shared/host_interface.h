#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include <stdint.h>
#include <stddef.h>
#include "host_interface_cmds.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    size_t (*read_fn)(int, uint8_t*, size_t);
    size_t (*write_fn)(int, const uint8_t*, size_t);
    void (*handler)(int, const HIP_Cmd_t*);
} HostIface_Callbacks_t;

void HostIface_Register(int dev, const HostIface_Callbacks_t* cbs);

int HostIface_PutData(int dev, uint16_t id, const uint8_t *buff, uint16_t len);
int HostIface_Send(int dev);

void HostIface_Listen(int dev);

#ifdef __cplusplus
}
#endif

#endif // _HOST_INTERFACE_H_
