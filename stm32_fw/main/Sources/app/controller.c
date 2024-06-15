#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "host_interface.h"
#include "host_interface_cmds.h"
#include "imu_reader.h"
#include "engine_control.h"

#include <string.h>

#define IMU_BUS							1
#define EC_BUS							1

#define MSG_QUEUE_LENGTH				4
#define MSG_ITEM_SIZE					sizeof(ControllerMessage_t)

extern int _dbg;

typedef enum
{
	ControllerMessageType_Meas,
	ControllerMessageType_Cmd,
} ControllerMessageType_t;

typedef struct
{
	ControllerMessageType_t type;
	union
	{
		MDI_output_t mdiData;
		HIP_Cmd_t cmd;
	} msgContent;

} ControllerMessage_t;

static StaticQueue_t _msgQueue;
static uint8_t _msgStorage[MSG_QUEUE_LENGTH * MSG_ITEM_SIZE];

struct
{
	QueueHandle_t hQueue;
	uint32_t msgCount;

	MDI_output_t lastMeas;

	uint32_t lastSend;
} g_controllerState;

/******************************************************************************/

static void _Controller_ProcessNewMeas(const MDI_output_t *mdiData);
static void _Controller_ProcessCommand(const HIP_Cmd_t *cmd);

static void _Controller_HandlePing(const HIP_Ping_t* cmd);
static void _Controller_HandleThrottle(const HIP_Throttle_t* cmd);

static void _Controller_SendMessages();

void _Controller_SendOrientation();

/******************************************************************************/

void Controller_Task()
{
	if ((g_controllerState.hQueue = xQueueCreateStatic(MSG_QUEUE_LENGTH, MSG_ITEM_SIZE,
		_msgStorage, &_msgQueue)) == NULL)
	{
		// TODO: handle error
	}

	int rc = HostIface_Start();
	if (rc)
	{
		// TODO: handle error
	}

	rc = EC_Init(EC_BUS);
	if (rc)
	{
		// TODO: handle error
	}


	IMU_Init(IMU_BUS);

	/*
	int n = 0;
	while (1)
	{
		float thr = fabs((1.0 + sin(n * 0.01)) / 2.0);
		for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
		{
			EC_SetThrottle(en, 0.4 + thr * 0.4);
		}

		n = (n + 1) % 628;

		vTaskDelay(100);
	}
	*/

	ControllerMessage_t msg;

	while (1)
	{
		if (xQueueReceive(g_controllerState.hQueue, &(msg),
			( TickType_t )0xFFFFFFFF ) == pdPASS )
		{
			switch (msg.type)
			{
				case ControllerMessageType_Meas:
					_Controller_ProcessNewMeas(&msg.msgContent.mdiData);
					break;
				case ControllerMessageType_Cmd:
					_Controller_ProcessCommand(&msg.msgContent.cmd);
				default:
					break;
			}

			_Controller_SendMessages();
		}
	}

	/*
	while (1)
	{
		// do nothing (yet)
		vTaskDelay(1000);
	}

	*/
}

void Controller_NewMeas(const MDI_output_t *mdiData)
{
	ControllerMessage_t msg;
	msg.type = ControllerMessageType_Meas;
	msg.msgContent.mdiData = *mdiData;

	xQueueSend(g_controllerState.hQueue, (void*)&msg, ( TickType_t ) 0 );
}

void Controller_NewCommand(const HIP_Cmd_t* cmd)
{
	ControllerMessage_t msg;
	msg.type = ControllerMessageType_Cmd;
	msg.msgContent.cmd = *cmd;

	xQueueSend(g_controllerState.hQueue, (void*)&msg, ( TickType_t ) 0 );
}

/******************************************************************************/

void _Controller_ProcessNewMeas(const MDI_output_t *mdiData)
{
	g_controllerState.msgCount++;
	g_controllerState.lastMeas = *mdiData;
}

static void _Controller_ProcessCommand(const HIP_Cmd_t *cmd)
{
	switch (cmd->header.cmd)
	{
		case HIP_MSG_PING:
			_Controller_HandlePing((HIP_Ping_t*)cmd);
			break;
		case HIP_MSG_THROTTLE:
			_Controller_HandleThrottle((HIP_Throttle_t*)cmd);
			break;
		default:
			_dbg = 1003;
			break;
	}
}

static void _Controller_HandlePing(const HIP_Ping_t* cmd)
{
	uint16_t rxSeq = cmd->payload.seqNumber;
	HostIface_PutData(HIP_MSG_PING, (uint8_t*)&rxSeq, sizeof(rxSeq));
	HostIface_Send();
	_dbg = 1004;
}

static void _Controller_HandleThrottle(const HIP_Throttle_t* cmd)
{
	if ((cmd->payload.flags & HIP_Throttle_Flags_Enable) == 0)
	{
		EC_Enable(0);
	}
	else
	{
		EC_Enable(1);
	}

	uint16_t cmdA = HIP_MSG_THROTTLE;
	HostIface_PutData(HIP_MSG_ACK, (uint8_t*)&cmdA, sizeof(cmdA));
	HostIface_Send();

	_dbg = 2001;
}

void _Controller_SendMessages()
{
	uint32_t now = xTaskGetTickCount();

	if ((now - g_controllerState.lastSend) > 0xffffffff)
	{
		_Controller_SendOrientation();
		g_controllerState.lastSend = now;

		HostIface_Send();
	}
}

void _Controller_SendOrientation()
{
	HIP_Payload_Orientation_t or;

	memcpy(or.rotation, g_controllerState.lastMeas.rotation, sizeof(or.rotation));
	memcpy(or.quaternion, g_controllerState.lastMeas.quaternion, sizeof(or.quaternion));
	memcpy(or.gravity, g_controllerState.lastMeas.gravity, sizeof(or.gravity));
	memcpy(or.linear_acceleration, g_controllerState.lastMeas.linear_acceleration,  sizeof(or.linear_acceleration));

	HostIface_PutData(HIP_MSG_STATE_OR, (uint8_t*)&or, sizeof(or));
}
