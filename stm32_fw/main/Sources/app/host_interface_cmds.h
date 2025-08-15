#ifndef _HOST_INTERFACE_STRUCTS_H_
#define _HOST_INTERFACE_STRUCTS_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define HIP_MAX_PAYLOAD 64

#define HIP_SYMBOL_M 'm'
#define HIP_SYMBOL_B 'b'

#define HIP_MSG_PING 0x0001

#define HIP_MSG_ACK 0x0100
#define HIP_MSG_NAK 0x0101

#define HIP_MSG_THROTTLE 0x0200

#define HIP_MSG_EM 0x0300
#define HIP_MSG_WM 0x0400
#define HIP_MSG_RESET_POS 0x0500
#define HIP_MSG_SET_PID 0x0600

#define HIP_MSG_PAT 0x0A00
#define HIP_MSG_ACC 0x0A01
#define HIP_MSG_MFX 0x0A02
#define HIP_MSG_DST 0x0A03
#define HIP_MSG_STB 0x0A04
#define HIP_MSG_PVT 0x0A05

#define HIP_MSG_CAL_ACC 0x0B01
#define HIP_MSG_CAL_GYRO 0x0B02

#define HIP_MSG_MON 0x0C00

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
        uint16_t cmd;
    } __attribute__((packed)) HIP_Payload_AckNak_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_AckNak_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_AckNak_t;

    typedef struct
    {
        float raw[3];
        float cal[3];
        float lin[3];
        float wld[3];
    } __attribute__((packed)) HIP_Payload_Acc_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_Acc_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_Acc_t;

    typedef struct
    {
        float scale[3 * 3];
        float bias[3];
        uint8_t flags; // for future use
    } __attribute__((packed)) HIP_Payload_Cal_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_Cal_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_Cal_t;

    typedef struct
    {
        float position[3];
        float rotation[3];
        float time;
    } __attribute__((packed)) HIP_Payload_PAT_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_PAT_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_PAT_t;

    typedef struct
    {
        float rotation[3];
        float rotation_dot[3];
        float pid[3];
        float thrustN[4];
    } __attribute__((packed)) HIP_Payload_STB_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_STB_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_STB_t;

    typedef struct
    {
        uint8_t flags;
        float throttle[4];
    } __attribute__((packed)) HIP_Payload_Throttle_t;

    typedef enum
    {
        HIP_Throttle_Flags_Enable = 1 << 0,
        HIP_Throttle_Flags_Eng1 = 1 << 1,
        HIP_Throttle_Flags_Eng2 = 1 << 2,
        HIP_Throttle_Flags_Eng3 = 1 << 3,
        HIP_Throttle_Flags_Eng4 = 1 << 4,
    } HIP_Throttle_Flags_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_Throttle_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_Throttle_t;

    typedef struct
    {
        uint16_t msgId;
        uint16_t msgPeriod;
    } __attribute__((packed)) HIP_Payload_EM_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_EM_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_EM_t;

    typedef struct
    {
        uint8_t imuMode;
        uint8_t fcMode;
    } __attribute__((packed)) HIP_Payload_WM_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_WM_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_WM_t;

    typedef struct
    {   struct
        {
            float kp[3];
            float kd[3];
            float ki[3];
        } att;

        float mass;
    } __attribute__((packed)) HIP_Payload_SetPID_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_SetPID_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_SetPID_t;

    typedef struct
    {
        float rotation[3];
        float gravity[3];
        float linear_acceleration[3];
    } __attribute__((packed)) HIP_Payload_MFX_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_MFX_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_MFX_t;

    typedef struct
    {
        float position[3];
        float velocity[3];
        float time;
    }  __attribute__((packed)) HIP_Payload_PVT_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_PVT_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_PVT_t;

    typedef struct
    {
        float throttle[4];
        float vbat;
        float ch1;
    } __attribute__((packed)) HIP_Payload_Mon_t;

    typedef struct
    {
        HIP_Header_t header;
        HIP_Payload_Mon_t payload;
        uint16_t crc;
    } __attribute__((packed)) HIP_Mon_t;

    typedef struct
    {
        HIP_Header_t header;
        uint8_t dummy;
        uint16_t crc;
    } __attribute__((packed)) HIP_ResetPos_t;
    typedef struct
    {
        HIP_Header_t header;
        uint8_t payload[HIP_MAX_PAYLOAD];
    } __attribute__((packed)) HIP_Cmd_t;

#ifdef __cplusplus
}
#endif

#endif // _HOST_INTERFACE_STRUCTS_H_