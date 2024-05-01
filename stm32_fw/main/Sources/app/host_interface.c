#include "host_interface.h"
#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"

#define HIP_SERIAL			0

#define HIP_MAX_PAYLOAD		256

#define HIP_SYMBOL_M		'm'
#define HIP_SYMBOL_B		'b'

#define HIP_MSG_PING		0x0001

#define HIP_STACK_SIZE		128

extern int _dbg;

typedef struct
{
	uint8_t m;
	uint8_t b;
	uint16_t cmd;
	uint16_t len;	
} __attribute__((packed)) HIP_Header_t;

typedef struct
{
	HIP_Header_t header;
	uint8_t payload[HIP_MAX_PAYLOAD];
} __attribute__((packed)) HIP_Cmd_t;

typedef struct
{
	HIP_Header_t header;
	uint16_t seqNumber;
	uint16_t crc;
} __attribute__((packed)) HIP_Ping_t;

/******************************************************************************/

static struct
{
	HIP_Cmd_t rxCmd;
	HIP_Cmd_t txCmd;
	uint16_t rxLen;
} _decoderCtx;

/******************************************************************************/

static TaskHandle_t _hListener = NULL;
static StaticTask_t _hipTaskBuffer;
StackType_t _hipTaskStack[HIP_STACK_SIZE / sizeof(StackType_t)];

static void _HostIface_Listen();
static void _HostIface_HandleCommand(const HIP_Cmd_t* cmd);
static void _HostIface_HandlePing(const HIP_Ping_t* cmd);

int HostIface_Start()
{
	if ((_hListener = xTaskCreateStatic((TaskFunction_t)_HostIface_Listen,
			(const char *)"HIP_LSTNR", HIP_STACK_SIZE / sizeof(StackType_t),
			NULL, 3, _hipTaskStack, &_hipTaskBuffer)) == NULL)
		return -1;	

	return 0;
}

int HostIface_SendData(uint16_t id, const uint8_t *buff, uint16_t len)
{
	_decoderCtx.txCmd.header.m = 'm';
	_decoderCtx.txCmd.header.b = 'b';
	_decoderCtx.txCmd.header.cmd = id;
	_decoderCtx.txCmd.header.len = len;
	for (int i = 0; i < len; i++)
		_decoderCtx.txCmd.payload[i] = buff[i];

	*(uint16_t*)&(_decoderCtx.txCmd.payload[len]) = 0xCACB; //replace with real CRC

	Serial_Write(HIP_SERIAL, (uint8_t*)&_decoderCtx.txCmd,
		sizeof(HIP_Header_t) + len + 2 /*CRC*/);

	_dbg = 1005;

	return 0;
}

/******************************************************************************/

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
					_decoderCtx.rxCmd.header.cmd = c;
					protoState = ProtoState_cmd1;
					break;
				case ProtoState_cmd1:
					_decoderCtx.rxCmd.header.cmd |= (uint16_t)c << 8;
					protoState = ProtoState_len0;
					break;
				case ProtoState_len0:
					_decoderCtx.rxCmd.header.len = c;
					protoState = ProtoState_len1;
					break;
				case ProtoState_len1:
					_decoderCtx.rxCmd.header.len |= (uint16_t)c << 8;
					protoState = ProtoState_payload;
					break;
				case ProtoState_payload:
					if ((_decoderCtx.rxLen < _decoderCtx.rxCmd.header.len) &&
						(_decoderCtx.rxLen < HIP_MAX_PAYLOAD - 2))
					{
						_decoderCtx.rxCmd.payload[_decoderCtx.rxLen++] = c;
						if (_decoderCtx.rxLen == _decoderCtx.rxCmd.header.len)
							protoState = ProtoState_crc0;
					}
					else
						protoState = ProtoState_m;
					break;
				case ProtoState_crc0:
					_decoderCtx.rxCmd.payload[_decoderCtx.rxLen++] = c;
					protoState = ProtoState_crc1;
					break;
				case ProtoState_crc1:
					_decoderCtx.rxCmd.payload[_decoderCtx.rxLen++] = c;

					_HostIface_HandleCommand(&_decoderCtx.rxCmd);

					protoState = ProtoState_m;
					break;
				case ProtoState_m:
				default:
					if (c == HIP_SYMBOL_M)
					{
						_decoderCtx.rxCmd.header.len = 0;
						_decoderCtx.rxCmd.header.cmd = 0;
						_decoderCtx.rxLen = 0;

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
		switch (cmd->header.cmd)
		{
			case HIP_MSG_PING:
				_HostIface_HandlePing((HIP_Ping_t*)cmd);
				break;
			default:
				_dbg = 1003;
				break;
		}
	}
	else
		_dbg = 1002;
}

static void _HostIface_HandlePing(const HIP_Ping_t* cmd)
{
	uint16_t rxSeq = cmd->seqNumber;
	HostIface_SendData(HIP_MSG_PING, (uint8_t*)&rxSeq, sizeof(rxSeq));
	_dbg = 1004;
}
