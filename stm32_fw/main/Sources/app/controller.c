#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "host_interface.h"
#include "host_interface_cmds.h"
#include "imu_reader.h"
#include "engine_control.h"

#include <math.h>

#include <string.h>

#define IMU_BUS 1
#define EC_BUS 1

#define MSG_QUEUE_LENGTH 4
#define MSG_ITEM_SIZE sizeof(ControllerMessage_t)

#define OM_MSG_POOL_SZ (sizeof(g_msgPool) / sizeof(g_msgPool[0]))

#define SNDR_STACK_SIZE 1024

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

/******************************************************************************/
struct
{
    QueueHandle_t hQueue;
    uint32_t msgCount;

    MDI_output_t lastMeas;

    uint32_t lastPing;

    MachineState_t mState;
} g_controllerState;

/******************************************************************************/

static void _Controller_ProcessNewMeas(const MDI_output_t *mdiData);
static void _Controller_ProcessCommand(const HIP_Cmd_t *cmd);

static void _Controller_HandlePing(const HIP_Ping_t *cmd);
static void _Controller_HandleThrottle(const HIP_Throttle_t *cmd);
static void _Controller_HandleEM(const HIP_EM_t *cmd);

static void _Controller_Process();

static void _Controller_SendMessages();

void _Controller_SendOrientation();

/******************************************************************************/

struct
{
    uint16_t msgId;
    uint16_t msgPeriod;
    uint32_t lastEmit;
    void (*emit)(void);
} g_msgPool[] =
    {
        {.msgId = HIP_MSG_IMU, .emit = _Controller_SendOrientation},
        {.msgId = HIP_MSG_PVT, .emit = 0},
};

/******************************************************************************/

/* TODO: support HW watchdog */

#define WD_STACK_SIZE 128

static TaskHandle_t _hWatchdog = NULL;
static StaticTask_t _watchdogBuffer;
StackType_t _watchdogStack[WD_STACK_SIZE / sizeof(StackType_t)];

static void _Watchdog_Task();

/******************************************************************************/

static TaskHandle_t _hSender = NULL;
static StaticTask_t _msgSenderBuffer;
StackType_t _msgSenderStack[SNDR_STACK_SIZE / sizeof(StackType_t)];

static void _Sender_Task();

/******************************************************************************/

void Controller_Task()
{
    g_controllerState.mState = MachineState_Disarmed;

    if ((_hWatchdog = xTaskCreateStatic((TaskFunction_t)_Watchdog_Task,
                                        (const char *)"WATCHDOG", WD_STACK_SIZE / sizeof(StackType_t),
                                        NULL, 3, _watchdogStack, &_watchdogBuffer)) == NULL)
    {
        // TODO: handle error
    }

    if ((_hSender = xTaskCreateStatic((TaskFunction_t)_Sender_Task,
                                      (const char *)"SNDR", SNDR_STACK_SIZE / sizeof(StackType_t),
                                      NULL, 3, _msgSenderStack, &_msgSenderBuffer)) == NULL)
    {
        // TODO: handle error
    }

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

    ControllerMessage_t msg;

    while (1)
    {
        if (xQueueReceive(g_controllerState.hQueue, &(msg),
                          (TickType_t)0xFFFFFFFF) == pdPASS)
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

            _Controller_Process();
        }
    }
}

/******************************************************************************/

static void _Controller_Process()
{
    static MachineState_t prevState = MachineState_Disarmed;

    if ((prevState != g_controllerState.mState) && (g_controllerState.mState == MachineState_Armed))
    {
        EC_Enable(0);

        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            EC_SetThrottle(en, 0);
        }

        EC_Enable(1);

        vTaskDelay(1000);

        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            EC_SetThrottle(en, 0.1);
        }
    }
    else if (g_controllerState.mState == MachineState_Armed)
    {
        float pitch = g_controllerState.lastMeas.rotation[1];
        float roll = g_controllerState.lastMeas.rotation[2];

        pitch = pitch / 180 * 3.145159;
        roll = roll / 180 * 3.145159;

        EC_SetThrottle(EC_Engine_1, 0.1 + fabs(pitch) * 30);
        EC_SetThrottle(EC_Engine_2, 0.1 + fabs(pitch) * 30);

        EC_SetThrottle(EC_Engine_3, 0.1 + fabs(roll) * 30);
        EC_SetThrottle(EC_Engine_4, 0.1 + fabs(roll) * 30);

        roll = roll;
    }

    prevState = g_controllerState.mState;
}

/******************************************************************************/

void _Sender_Task()
{
    while (1)
    {
        _Controller_SendMessages();
        vTaskDelay(100);
    }
}

/******************************************************************************/

void Controller_NewMeas(const MDI_output_t *mdiData)
{
    ControllerMessage_t msg;
    msg.type = ControllerMessageType_Meas;
    msg.msgContent.mdiData = *mdiData;

    xQueueSend(g_controllerState.hQueue, (void *)&msg, (TickType_t)0);
}

void Controller_NewCommand(const HIP_Cmd_t *cmd)
{
    ControllerMessage_t msg;
    msg.type = ControllerMessageType_Cmd;
    msg.msgContent.cmd = *cmd;

    xQueueSend(g_controllerState.hQueue, (void *)&msg, (TickType_t)0);
}

void Controller_HandleFatal()
{
    g_controllerState.mState = MachineState_HardFault;
    EC_Enable(0);
}

MachineState_t GetMachineState()
{
    return g_controllerState.mState;
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
        _Controller_HandlePing((HIP_Ping_t *)cmd);
        break;
    case HIP_MSG_THROTTLE:
        _Controller_HandleThrottle((HIP_Throttle_t *)cmd);
        break;
    case HIP_MSG_EM:
        _Controller_HandleEM((HIP_EM_t *)cmd);
        break;
    default:
        _dbg = 1003;
        break;
    }
}

static void _Controller_HandlePing(const HIP_Ping_t *cmd)
{
    g_controllerState.mState = MachineState_Armed;

    uint16_t rxSeq = cmd->payload.seqNumber;
    HostIface_PutData(HIP_MSG_PING, (uint8_t *)&rxSeq, sizeof(rxSeq));

    g_controllerState.lastPing = xTaskGetTickCount();
}

static void _Controller_HandleThrottle(const HIP_Throttle_t *cmd)
{
    if ((cmd->payload.flags & HIP_Throttle_Flags_Enable) == 0)
    {
        EC_Enable(0);
        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            EC_SetThrottle(en, 0.0);
        }
    }
    else
    {
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng1) != 0)
            EC_SetThrottle(EC_Engine_1, cmd->payload.throttle[0]);
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng2) != 0)
            EC_SetThrottle(EC_Engine_2, cmd->payload.throttle[1]);
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng3) != 0)
            EC_SetThrottle(EC_Engine_3, cmd->payload.throttle[2]);
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng4) != 0)
            EC_SetThrottle(EC_Engine_4, cmd->payload.throttle[3]);

        EC_Enable(1);
    }

    uint16_t cmdA = HIP_MSG_THROTTLE;
    HostIface_PutData(HIP_MSG_ACK, (uint8_t *)&cmdA, sizeof(cmdA));

    _dbg = 2001;
}

static void _Controller_HandleEM(const HIP_EM_t *cmd)
{
    for (int i = 0; i < OM_MSG_POOL_SZ; i++)
    {
        if (g_msgPool[i].msgId == cmd->payload.msgId)
            g_msgPool[i].msgPeriod = cmd->payload.msgPeriod;
    }

    uint16_t cmdA = HIP_MSG_EM;
    HostIface_PutData(HIP_MSG_ACK, (uint8_t *)&cmdA, sizeof(cmdA));

    _dbg = 2002;
}

void _Controller_SendMessages()
{
    uint32_t now = xTaskGetTickCount();
    now /= 100;

    for (int i = 0; i < OM_MSG_POOL_SZ; i++)
    {
        if ((g_msgPool[i].msgPeriod != 0) && (now - g_msgPool[i].lastEmit) > g_msgPool[i].msgPeriod)
        {
            g_msgPool[i].emit();
            g_msgPool[i].lastEmit = now;
        }
    }

    HostIface_Send();
}

void _Controller_SendOrientation()
{
    HIP_Payload_IMU_t or ;

    memcpy(or.rotation, g_controllerState.lastMeas.rotation, sizeof(or.rotation));
    memcpy(or.gravity, g_controllerState.lastMeas.gravity, sizeof(or.gravity));
    memcpy(or.linear_acceleration, g_controllerState.lastMeas.linear_acceleration, sizeof(or.linear_acceleration));

    HostIface_PutData(HIP_MSG_IMU, (uint8_t *)& or, sizeof(or));
}

/******************************************************************************/

static void _Watchdog_Task() // TODO: enable normal watchdog
{
    while (1)
    {
        uint32_t now = xTaskGetTickCount();

        if ((now - g_controllerState.lastPing) > 2000)
        {
            Controller_HandleFatal();
        }

        vTaskDelay(500);
    }
}
