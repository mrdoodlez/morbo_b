#ifndef _HOST_INTERFACE_STRUCTS_H_
#define _HOST_INTERFACE_STRUCTS_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define HIP_MAX_PAYLOAD		256

#define HIP_SYMBOL_M		'm'
#define HIP_SYMBOL_B		'b'

#define HIP_MSG_PING		0x0001

#define HIP_MSG_ACK			0x0100
#define HIP_MSG_NAK			0x0101


/*
*
*
*
*/


#define HIP_MSG_STATE_OR	0x0A00

/******************************************************************************/

typedef struct
{
	uint8_t m;
	uint8_t b;
	uint16_t cmd;
	uint16_t len;	
} __attribute__((packed)) HIP_Header_t;

typedef struct
{
	uint16_t seqNumber;
} __attribute__((packed)) HIP_Payload_Ping_t;

typedef struct
{
	HIP_Header_t header;
	HIP_Payload_Ping_t payload;	
	uint16_t crc;
} __attribute__((packed)) HIP_Ping_t;

typedef struct
{
	float rotation[3];            /* 6 axes yaw, pitch and roll */
	float quaternion[4];          /* 6 axes quaternion */
	float gravity[3];             /* 6 axes device frame gravity */
	float linear_acceleration[3]; /* 6 axes device frame linear acceleration */
} __attribute__((packed)) HIP_Payload_Orientation_t;

typedef struct
{
	HIP_Header_t header;
	HIP_Payload_Orientation_t payload;	
	uint16_t crc;
} __attribute__((packed)) HIP_Orientation_t;

#ifdef __cplusplus
}
#endif

#endif // _HOST_INTERFACE_STRUCTS_H_