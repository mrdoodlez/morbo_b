#include "controller.h"
#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"

static uint8_t txBuffer[] = "Hello from STM32!\r\n";

void Controller_Task()
{
	while (1)
	{
		BSP_LED_Toggle(LED2);

		Serial_Write(0, txBuffer, sizeof(txBuffer) - 1);

		vTaskDelay(1000);
	}
}