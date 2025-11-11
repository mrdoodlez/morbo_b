#include "host_interface.h"
#include "host_interface_cmds.h"
#include "serial.h"

#define HIP_TX_SIZE 1024
#define COMM_CNT    2
struct
{
    HIP_Cmd_t rxCmd;
    uint16_t rxLen;
    uint32_t numRxBytes;
} g_decoderCtx[COMM_CNT];

struct
{
    uint8_t txBuffer[HIP_TX_SIZE];
    uint8_t txLen;
    uint32_t numTxBytes;
} g_coderCtx[COMM_CNT];

/******************************************************************************/

static void _HostIface_HandleCommand(const HIP_Cmd_t *cmd);

int HostIface_PutData(int dev, uint16_t id, const uint8_t *buff, uint16_t len)
{
    HIP_Cmd_t *txCmd = (HIP_Cmd_t *)(g_coderCtx[dev].txBuffer + g_coderCtx[dev].txLen);

    txCmd->header.m = 'm';
    txCmd->header.b = 'b';
    txCmd->header.cmd = id;
    txCmd->header.len = len;
    for (int i = 0; i < len; i++)
        txCmd->payload[i] = buff[i];

    *(uint16_t *)&(txCmd->payload[len]) = 0xCACB; // replace with real CRC

    g_coderCtx[dev].txLen += (sizeof(HIP_Header_t) + len + 2 /*CRC*/);

    return 0;
}

int HostIface_Send(int dev)
{
    if (g_coderCtx[dev].txLen > 0)
    {
        int sz = g_coderCtx[dev].txLen;
        Serial_Write(dev, g_coderCtx[dev].txBuffer, sz);
        g_coderCtx[dev].numTxBytes += sz;
        g_coderCtx[dev].txLen = 0;
    }

    return 0;
}

void HostIface_Listen(int dev)
{
    enum
    {
        ProtoState_m,
        ProtoState_b,
        ProtoState_len0,
        ProtoState_len1,
        ProtoState_cmd0,
        ProtoState_cmd1,
        ProtoState_payload,
        ProtoState_crc0,
        ProtoState_crc1,
    } protoState = ProtoState_m;

    while (1)
    {
        uint8_t c;
        if (Serial_Read(dev, &c, 1) == 1)
        {
            g_decoderCtx[dev].numRxBytes++;
            switch (protoState)
            {
            case ProtoState_b:
                if (c == HIP_SYMBOL_B)
                    protoState = ProtoState_cmd0;
                else
                    protoState = ProtoState_m;
                break;
            case ProtoState_cmd0:
                g_decoderCtx[dev].rxCmd.header.cmd = c;
                protoState = ProtoState_cmd1;
                break;
            case ProtoState_cmd1:
                g_decoderCtx[dev].rxCmd.header.cmd |= (uint16_t)c << 8;
                protoState = ProtoState_len0;
                break;
            case ProtoState_len0:
                g_decoderCtx[dev].rxCmd.header.len = c;
                protoState = ProtoState_len1;
                break;
            case ProtoState_len1:
                g_decoderCtx[dev].rxCmd.header.len |= (uint16_t)c << 8;
                protoState = ProtoState_payload;
                break;
            case ProtoState_payload:
                if ((g_decoderCtx[dev].rxLen < g_decoderCtx[dev].rxCmd.header.len) &&
                    (g_decoderCtx[dev].rxLen < HIP_MAX_PAYLOAD - 2))
                {
                    g_decoderCtx[dev].rxCmd.payload[g_decoderCtx[dev].rxLen++] = c;
                    if (g_decoderCtx[dev].rxLen == g_decoderCtx[dev].rxCmd.header.len)
                        protoState = ProtoState_crc0;
                }
                else
                    protoState = ProtoState_m;
                break;
            case ProtoState_crc0:
                g_decoderCtx[dev].rxCmd.payload[g_decoderCtx[dev].rxLen++] = c;
                protoState = ProtoState_crc1;
                break;
            case ProtoState_crc1:
                g_decoderCtx[dev].rxCmd.payload[g_decoderCtx[dev].rxLen++] = c;

                _HostIface_HandleCommand(&g_decoderCtx[dev].rxCmd);

                protoState = ProtoState_m;
                break;
            case ProtoState_m:
            default:
                if (c == HIP_SYMBOL_M)
                {
                    g_decoderCtx[dev].rxCmd.header.len = 0;
                    g_decoderCtx[dev].rxCmd.header.cmd = 0;
                    g_decoderCtx[dev].rxLen = 0;

                    protoState = ProtoState_b;
                }
                break;
            }
        }
    }
}

static void _HostIface_HandleCommand(const HIP_Cmd_t *cmd)
{
    uint16_t crc = *(uint16_t *)&(cmd->payload[cmd->header.len]);

    if (crc == 0xCACB) // TODO: replace with real check
    {
        Controller_NewCommand(cmd);
    } 
}
