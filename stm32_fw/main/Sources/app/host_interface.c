#include "host_interface.h"
#include "host_interface_cmds.h"
#include "controller.h"
#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define HIP_SERIAL				0

#define HIP_STACK_SIZE			1024
#define HIP_TX_SIZE 			1024

#define MSG_QUEUE_LENGTH		4
#define MSG_ITEM_SIZE			sizeof(HIP_Cmd_t)

extern int _dbg;

/******************************************************************************/
struct
{
	QueueHandle_t hQueue;
	uint32_t msgCount;

	HIP_Cmd_t rxCmd;
	uint16_t rxLen;

	uint8_t txBuffer[HIP_TX_SIZE];
	uint8_t txLen;

	SemaphoreHandle_t txSem;
} g_decoderCtx;

/******************************************************************************/

static TaskHandle_t _hListener = NULL;
static StaticTask_t _hipListenerBuffer;
StackType_t _hipListenerStack[HIP_STACK_SIZE / sizeof(StackType_t)];

static TaskHandle_t _hSender = NULL;
static StaticTask_t _hipsenderBuffer;
StackType_t _hipSenderStack[HIP_STACK_SIZE / sizeof(StackType_t)];

static StaticSemaphore_t _txSemBuffer;

static void _HostIface_Listen();
static void _HostIface_Send();

static void _HostIface_HandleCommand(const HIP_Cmd_t* cmd);

int HostIface_Start()
{
	g_decoderCtx.txSem = xSemaphoreCreateBinaryStatic(&_txSemBuffer);

	if ((_hSender = xTaskCreateStatic((TaskFunction_t)_HostIface_Send,
			(const char *)"HIP_SNDER", HIP_STACK_SIZE / sizeof(StackType_t),
			NULL, 3, _hipSenderStack, &_hipsenderBuffer)) == NULL)
		return -20;

	if ((_hListener = xTaskCreateStatic((TaskFunction_t)_HostIface_Listen,
			(const char *)"HIP_LSTNR", HIP_STACK_SIZE / sizeof(StackType_t),
			NULL, 3, _hipListenerStack, &_hipListenerBuffer)) == NULL)
		return -30;

	return 0;
}

int HostIface_PutData(uint16_t id, const uint8_t *buff, uint16_t len)
{
	HIP_Cmd_t *txCmd = (HIP_Cmd_t*)(g_decoderCtx.txBuffer + g_decoderCtx.txLen);

	txCmd->header.m = 'm';
	txCmd->header.b = 'b';
	txCmd->header.cmd = id;
	txCmd->header.len = len;
	for (int i = 0; i < len; i++)
		txCmd->payload[i] = buff[i];

	*(uint16_t*)&(txCmd->payload[len]) = 0xCACB; //replace with real CRC

	g_decoderCtx.txLen += (sizeof(HIP_Header_t) + len + 2 /*CRC*/);

	return 0;
}

int HostIface_Send()
{
	xSemaphoreGive(g_decoderCtx.txSem);
	return 0;
}

/******************************************************************************/

static void _HostIface_Send()
{
	while (1)
	{
		if (xSemaphoreTake(g_decoderCtx.txSem, 0xFFFFFFFF) == pdTRUE)
		{
			Serial_Write(HIP_SERIAL, g_decoderCtx.txBuffer, g_decoderCtx.txLen);
			g_decoderCtx.txLen = 0;
		}
		else
		{
			// TODO: handle error
		}
	}
}

static void _HostIface_Listen()
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
	}
	protoState = ProtoState_m;

	while (1)
	{
		uint8_t c;
		if (Serial_Read(HIP_SERIAL, &c, 1) == 1)
		{
			switch(protoState)
			{
				case ProtoState_b:
					if (c == HIP_SYMBOL_B)
						protoState = ProtoState_cmd0;
					break;
				case ProtoState_cmd0:
					g_decoderCtx.rxCmd.header.cmd = c;
					protoState = ProtoState_cmd1;
					break;
				case ProtoState_cmd1:
					g_decoderCtx.rxCmd.header.cmd |= (uint16_t)c << 8;
					protoState = ProtoState_len0;
					break;
				case ProtoState_len0:
					g_decoderCtx.rxCmd.header.len = c;
					protoState = ProtoState_len1;
					break;
				case ProtoState_len1:
					g_decoderCtx.rxCmd.header.len |= (uint16_t)c << 8;
					protoState = ProtoState_payload;
					break;
				case ProtoState_payload:
					if ((g_decoderCtx.rxLen < g_decoderCtx.rxCmd.header.len) &&
						(g_decoderCtx.rxLen < HIP_MAX_PAYLOAD - 2))
					{
						g_decoderCtx.rxCmd.payload[g_decoderCtx.rxLen++] = c;
						if (g_decoderCtx.rxLen == g_decoderCtx.rxCmd.header.len)
							protoState = ProtoState_crc0;
					}
					else
						protoState = ProtoState_m;
					break;
				case ProtoState_crc0:
					g_decoderCtx.rxCmd.payload[g_decoderCtx.rxLen++] = c;
					protoState = ProtoState_crc1;
					break;
				case ProtoState_crc1:
					g_decoderCtx.rxCmd.payload[g_decoderCtx.rxLen++] = c;

					_HostIface_HandleCommand(&g_decoderCtx.rxCmd);

					protoState = ProtoState_m;
					break;
				case ProtoState_m:
				default:
					if (c == HIP_SYMBOL_M)
					{
						g_decoderCtx.rxCmd.header.len = 0;
						g_decoderCtx.rxCmd.header.cmd = 0;
						g_decoderCtx.rxLen = 0;

						protoState = ProtoState_b;
					}
					break;
			}
		}
	}
}

static void _HostIface_HandleCommand(const HIP_Cmd_t* cmd)
{
	uint16_t crc = *(uint16_t*)&(cmd->payload[cmd->header.len]);

	if (crc == 0xCACB) // TODO: replace with real check
	{
		Controller_NewCommand(cmd);
	}
	else
		_dbg = 1002;
}

