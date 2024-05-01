#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "host_interface.h"
#include "pca9685.h"
#include "imu_reader.h"

#define PCA9685_BUS						1
#define PCA9685_ADDR					0x80

#define IMU_BUS							1

#define CONTROLLER_MESSAGE_CMD_XX		0x0000
#define CONTROLLER_MESSAGE_DI_MEAS		0x0100

#define MSG_QUEUE_LENGTH				4
#define MSG_ITEM_SIZE					sizeof(ControllerMessage_t)

typedef struct
{
	uint16_t id;
	union
	{
		MDI_output_t mdiData;
		uint32_t dummy;
	} msgContent;

} ControllerMessage_t;

static StaticQueue_t _msgQueue;
uint8_t _msgStorage[MSG_QUEUE_LENGTH * MSG_ITEM_SIZE];

static struct
{
	QueueHandle_t hQueue;
	uint32_t msgCount;

	MDI_output_t lastMeas;
	uint32_t lastSend;
} g_controllerState;

/******************************************************************************/

static void _Controller_ProcessNewMeas(MDI_output_t *mdiData);
static void _Controller_SendMessages();

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

	PCA9685_Init(PCA9685_BUS, PCA9685_ADDR, 0);

	PCA9685_SetPWMFreq(1600);

	IMU_Init(IMU_BUS);

	ControllerMessage_t msg;

	while (1)
	{
		if (xQueueReceive(g_controllerState.hQueue, &(msg),
			( TickType_t )0xFFFFFFFF ) == pdPASS )
		{
			switch (msg.id)
			{
				case CONTROLLER_MESSAGE_CMD_XX:
					break;
				case CONTROLLER_MESSAGE_DI_MEAS:
					_Controller_ProcessNewMeas(&msg.msgContent.mdiData);
					break;
				default:
					break;
			}

			_Controller_SendMessages();
		}
	}

	/*

	while (1)
	{
		for (uint16_t i = 0; i < 4096; i +=  8)
		{
			for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++)
			{
				PCA9685_SetPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
			}
		}

		vTaskDelay(100);
	}

	while (1)
	{
		// do nothing (yet)
		vTaskDelay(1000);
	}

	*/
}

void Controller_NewMeas(MDI_output_t *mdiData)
{
	ControllerMessage_t msg;
	msg.id = CONTROLLER_MESSAGE_DI_MEAS;
	msg.msgContent.mdiData = *mdiData;

	xQueueSend(g_controllerState.hQueue, (void*)&msg, ( TickType_t ) 0 );
}

/******************************************************************************/

void _Controller_ProcessNewMeas(MDI_output_t *mdiData)
{
	g_controllerState.msgCount++;
	g_controllerState.lastMeas = *mdiData;
}

void _Controller_SendMessages()
{
	//uint32_t now;

}
