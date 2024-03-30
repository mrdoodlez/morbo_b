#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "host_interface.h"

void Controller_Task()
{
	int rc = HostIface_Start();
	if (rc)
	{
		// TODO: handle error
	}

	while (1)
	{
		// do nothing (yet)
		vTaskDelay(1000);
	}
}