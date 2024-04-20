#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "host_interface.h"
#include "pca9685.h"
#include "imu_reader.h"

#define PCA9685_BUS			1
#define PCA9685_ADDR		0x80

#define IMU_BUS				1

int _cmpltCount = 0;

void Controller_Task()
{
	int rc = HostIface_Start();
	if (rc)
	{
		// TODO: handle error
	}

	PCA9685_Init(PCA9685_BUS, PCA9685_ADDR, 0);

	IMU_Init(IMU_BUS);

	while (1)
	{
		IMU_Read();
		vTaskDelay(10);
	}

	while (1)
	{
		// do nothing (yet)
		vTaskDelay(1000);
	}

	PCA9685_SetPWMFreq(1600);

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
}