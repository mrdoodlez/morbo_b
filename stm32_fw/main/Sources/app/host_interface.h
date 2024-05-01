#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	float rotation[3];            /* 6 axes yaw, pitch and roll */
	float quaternion[4];          /* 6 axes quaternion */
	float gravity[3];             /* 6 axes device frame gravity */
	float linear_acceleration[3]; /* 6 axes device frame linear acceleration */
} __attribute__((packed)) HIP_Payload_Orientation_t;
;

int HostIface_Start();
int HostIface_SendData(uint16_t id, const uint8_t *buff, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // _HOST_INTERFACE_H_