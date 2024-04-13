#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "host_interface.h"

#include "i2c.h"

int _cmpltCount = 0;

void Controller_Task()
{
	int rc = HostIface_Start();
	if (rc)
	{
		// TODO: handle error
	}

	uint8_t regAddr = 0;
	uint8_t regVal  = 0;
	uint8_t devAddr = 0xE0;

	while (1)
	{
		I2C_Write(1, devAddr, regAddr, I2C_RegAddrLen_8, &regVal, 1);

		_cmpltCount++;

		vTaskDelay(100);
	}

	while (1)
	{
		// do nothing (yet)
		vTaskDelay(1000);
	}
}