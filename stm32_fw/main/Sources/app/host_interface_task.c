#include "host_interface_task.h"
#include "host_interface.h"
#include "FreeRTOS.h"
#include "task.h"

#define HIP_STACK_SIZE 1024

static TaskHandle_t _hListener = NULL;
static StaticTask_t _hipListenerBuffer;
StackType_t _hipListenerStack[HIP_STACK_SIZE / sizeof(StackType_t)];

int HostIface_Start(int dev)
{
    // g_decoderCtx.txSem = xSemaphoreCreateBinaryStatic(&_txSemBuffer);

    if ((_hListener = xTaskCreateStatic((TaskFunction_t)HostIface_Listen,
                                        (const char *)"HIP_LSTNR", HIP_STACK_SIZE / sizeof(StackType_t),
                                        (void*)dev, 3, _hipListenerStack, &_hipListenerBuffer)) == NULL)
        return -30;

    return 0;
}