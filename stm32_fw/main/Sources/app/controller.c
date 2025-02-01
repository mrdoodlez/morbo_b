#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "host_interface.h"
#include "host_interface_cmds.h"
#include "imu_reader.h"
#include "engine_control.h"
#include "scenarios.h"
#include "motion_di.h"

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
    MachineState_Disarmed,
    MachineState_Armed,
    MachineState_Debug,
    MachineState_HardFault,
} MachineState_t;

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
    float pwm[4];
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
void _Controller_SendPAT();

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
        {.msgId = HIP_MSG_PAT, .emit = _Controller_SendPAT},
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
    vTaskDelay(1000);

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

    /*
    rc = EC_Init(EC_BUS);
    if (rc)
    {
        // TODO: handle error
    }
    */

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
        /*
        EC_Enable(0);

        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            EC_SetThrottle(en, 0);
        }

        EC_Enable(1);

        vTaskDelay(1000);

        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            g_controllerState.pwm[en] = 0.1;
            EC_SetThrottle(en, g_controllerState.pwm[en]);
        }
        */
    }
    else if (g_controllerState.mState == MachineState_Armed)
    {
        struct {
            uint64_t time;
            MDI_output_t meas;
        } meas;

        meas.time = Controller_GetUS();
        meas.meas = g_controllerState.lastMeas;

        FlightScenario_SetInputs(FlightScenario_Input_Meas, (void *)&meas);

        ControlOutputs_t output;
        FlightScenario_Result_t fcRes = FlightScenario(&output);

        if (fcRes == FlightScenario_Result_OK)
        {
            for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
            {
                g_controllerState.pwm[en] = output.pwm[en];
                //EC_SetThrottle(en, g_controllerState.pwm[en]);
            }
        }
        else if (fcRes == FlightScenario_Result_Error)
        {
            g_controllerState.mState = MachineState_HardFault;
        }
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
    // EC_Enable(0);
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
        // EC_Enable(0);
    }
    else
    {
        // EC_Enable(1);

        float throttles[4] = {-1, -1, -1, -1};
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng1) != 0)
            throttles[0] = cmd->payload.throttle[0];
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng2) != 0)
            throttles[1] = cmd->payload.throttle[1];
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng3) != 0)
            throttles[2] = cmd->payload.throttle[2];
        if ((cmd->payload.flags & HIP_Throttle_Flags_Eng4) != 0)
            throttles[3] = cmd->payload.throttle[3];

        FlightScenario_SetInputs(FlightScenario_Input_DebugPwms, &(throttles[0]));
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
    HIP_Payload_IMU_t or;

    memcpy(or.rotation, g_controllerState.lastMeas.rotation, sizeof(or.rotation));
    memcpy(or.gravity, g_controllerState.lastMeas.gravity, sizeof(or.gravity));
    memcpy(or.linear_acceleration, g_controllerState.lastMeas.linear_acceleration, sizeof(or.linear_acceleration));

    HostIface_PutData(HIP_MSG_IMU, (uint8_t *)& or, sizeof(or));
}

void _Controller_SendPAT()
{
    FlightScenario_PAT_t pat;
    FlightScenario_GetPAT(&pat);

    HIP_Payload_PAT_t ppat;
    memcpy(ppat.position, pat.p, sizeof(ppat.position));
    memcpy(ppat.rotation, pat.r, sizeof(ppat.rotation));
    ppat.time = pat.time;

    HostIface_PutData(HIP_MSG_PAT, (uint8_t *)&ppat, sizeof(ppat));
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

/******************************************************************************/

uint64_t Controller_GetUS()
{
    return 1000LL * xTaskGetTickCount();
}
