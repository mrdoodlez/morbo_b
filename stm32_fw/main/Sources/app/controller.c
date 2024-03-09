#include "controller.h"
#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"

xTaskHandle hRdTask;

static const char hexes[] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
};

uint8_t cmd = 0x00;

static void Reader_Task();

void Controller_Task()
{	
	xTaskCreate((TaskFunction_t)Reader_Task, (const char *)"READER",
		100, NULL, 3, &hRdTask);

	while (1)
	{
		uint8_t str[4];
		str[0] = hexes[cmd >> 4];
		str[1] = hexes[cmd & 0xf];
		str[2] = '\r';
		str[3] = '\n';

		Serial_Write(0, str, sizeof(str));

		vTaskDelay(1000);
	}
}

static void Reader_Task()
{
	while (1)
	{		
		if (Serial_Read(0, &cmd, sizeof(cmd)) == sizeof(cmd))
		{
			BSP_LED_Toggle(LED2);
		}
	}
}