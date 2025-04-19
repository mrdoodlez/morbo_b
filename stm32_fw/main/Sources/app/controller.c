#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "host_interface.h"
#include "host_interface_cmds.h"
#include "imu_reader.h"
#include "engine_control.h"
#include "scenarios.h"
#include "motion_fx.h"
#include "monitor.h"
#include "timer.h"

#include <math.h>

#include <string.h>

#define MSG_QUEUE_LENGTH 4
#define MSG_ITEM_SIZE sizeof(ControllerMessage_t)

#define OM_MSG_POOL_SZ (sizeof(g_msgPool) / sizeof(g_msgPool[0]))

#define SNDR_STACK_SIZE 1024

extern int _dbg;

typedef enum
{
    MachineState_Disarmed,
    MachineState_Armed,
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
        MFX_output_t mdiData;
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

    MFX_output_t lastMeas;

    uint32_t lastPing;

    MachineState_t mState;
    float pwm[4];

    float vbat;
    float ch1;

    uint64_t rt;
} g_controllerState;

/******************************************************************************/

static void _Controller_ProcessNewMeas(const MFX_output_t *mdiData);
static void _Controller_ProcessCommand(const HIP_Cmd_t *cmd);

static void _Controller_HandlePing(const HIP_Ping_t *cmd);
static void _Controller_HandleThrottle(const HIP_Throttle_t *cmd);
static void _Controller_HandleEM(const HIP_EM_t *cmd);
static void _Controller_HandleWM(const HIP_WM_t *cmd);
static void _Controller_HandleResetPos(const HIP_ResetPos_t *cmd);

static void _Controller_Process(uint8_t newMeas);

static void _Controller_SendMessages();

static void _Controller_SendAccAxes();
static void _Controller_SendAccCal();

static void _Controller_SendMFX();
static void _Controller_SendLAV();

static void _Controller_SendPAT();

static void _Controller_SendMon();

/******************************************************************************/

struct
{
    uint16_t msgId;
    uint16_t msgPeriod;
    uint32_t lastEmit;
    void (*emit)(void);
} g_msgPool[] =
    {
        {.msgId = HIP_MSG_PAT, .emit = _Controller_SendPAT},
        {.msgId = HIP_MSG_ACC, .emit = _Controller_SendAccAxes},
        {.msgId = HIP_MSG_CAL_ACC, .emit = _Controller_SendAccCal},
        {.msgId = HIP_MSG_MFX, .emit = _Controller_SendMFX},
        {.msgId = HIP_MSG_LAV, .emit = _Controller_SendLAV},
        {.msgId = HIP_MSG_MON, .emit = _Controller_SendMon},
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

    rc = EC_Init(EC_BUS);
    if (rc)
    {
        // TODO: handle error
    }

    IMU_Init(IMU_BUS);

    IMU_SetMode(IMU_Mode_Fusion); // TODO: comment it!

    FlightScenario_Init(FUSION_FREQ);

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

            _Controller_Process(msg.type == ControllerMessageType_Meas);
        }
    }
}

/******************************************************************************/

static void _Controller_Process(uint8_t newMeas)
{
    static MachineState_t prevState = MachineState_Disarmed;

    Monitor_Update();

    g_controllerState.vbat = Monitor_GetVbat();
    g_controllerState.ch1 = Monitor_GetCh1();

    g_controllerState.rt = Controller_GetUS();

    if ((prevState == MachineState_Disarmed)
        && (g_controllerState.mState == MachineState_Armed))
    {
        EC_Enable(0);

        for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
        {
            EC_SetThrottle(en, 0, 1);
        }

        EC_Enable(1);
    }
    else if ((g_controllerState.mState == MachineState_Armed) && (newMeas != 0))
    {
        struct
        {
            uint64_t time;
            MFX_output_t meas;
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
                EC_SetThrottle(en, g_controllerState.pwm[en], 0);
            }
        }
        else if (fcRes == FlightScenario_Result_Error)
        {
            Controller_HandleFatal();
        }
    }
    else if (g_controllerState.mState == MachineState_Disarmed)
    {
        EC_Enable(0);
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

void Controller_NewMeas(const MFX_output_t *mdiData)
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

/******************************************************************************/

void _Controller_ProcessNewMeas(const MFX_output_t *mdiData)
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
    case HIP_MSG_WM:
        _Controller_HandleWM((HIP_WM_t *)cmd);
        break;
    case HIP_MSG_RESET_POS:
        _Controller_HandleResetPos((HIP_ResetPos_t *)cmd);
        break;
    default:
        _dbg = 1003;
        break;
    }
}

static void _Controller_HandlePing(const HIP_Ping_t *cmd)
{
    if (g_controllerState.mState != MachineState_HardFault)
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
    }
    else
    {
        EC_Enable(1);

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

static void _Controller_HandleWM(const HIP_WM_t *cmd)
{
    IMU_SetMode(cmd->payload.imuMode);
    FlightScenario_SetScenario(cmd->payload.fcMode);

    uint16_t cmdA = HIP_MSG_WM;
    HostIface_PutData(HIP_MSG_ACK, (uint8_t *)&cmdA, sizeof(cmdA));

    _dbg = 2003;
}

static void _Controller_HandleResetPos(const HIP_ResetPos_t *cmd)
{
    FlightScenario_ResetPos();

    uint16_t cmdA = HIP_MSG_RESET_POS;
    HostIface_PutData(HIP_MSG_ACK, (uint8_t *)&cmdA, sizeof(cmdA));

    _dbg = 2004;
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

void _Controller_SendAccAxes()
{
    HIP_Payload_Acc_t acc;
    Vec3D_t raw;
    Vec3D_t cal;

    IMU_GetAxes(IMU_Sensor_Acc, &raw, &cal);
    memcpy(&acc.raw, &raw.x, sizeof(acc.raw));
    memcpy(&acc.cal, &cal.x, sizeof(acc.cal));

    HostIface_PutData(HIP_MSG_ACC, (uint8_t *)&acc, sizeof(acc));
}

void _Controller_SendAccCal()
{

    IMU_CalData_t imu;
    uint8_t calStatus;
    IMU_GetCalData(IMU_Sensor_Acc, &imu, &calStatus);

    HIP_Payload_Cal_t cal;
    memcpy(&cal.scale, &imu.scale.r, sizeof(cal.scale));
    memcpy(&cal.bias, &imu.ofsset.x, sizeof(cal.bias));
    cal.flags = calStatus;

    HostIface_PutData(HIP_MSG_CAL_ACC, (uint8_t *)&cal, sizeof(cal));
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

void _Controller_SendMFX()
{
    HIP_Payload_MFX_t mfx;

    memcpy(mfx.linear_acceleration,
           g_controllerState.lastMeas.linear_acceleration, sizeof(mfx.linear_acceleration));
    memcpy(mfx.rotation,
           g_controllerState.lastMeas.rotation, sizeof(mfx.rotation));
    memcpy(mfx.gravity, g_controllerState.lastMeas.gravity,
           sizeof(mfx.gravity));

    HostIface_PutData(HIP_MSG_MFX, (uint8_t *)&mfx, sizeof(mfx));
}

void _Controller_SendLAV()
{
    Vec3D_t la = *(Vec3D_t *)(g_controllerState.lastMeas.linear_acceleration);
    FS_ScaleVec(GRAVITY, &la);

    FS_State_t s;
    FlightScenario_GetState(&s);

    HIP_Payload_LAV_t lav;
    memcpy(lav.linear_acceleration, &la, sizeof(lav.linear_acceleration));
    memcpy(lav.world_acceleration, s.a, sizeof(lav.world_acceleration));
    memcpy(lav.velocity, s.v, sizeof(lav.velocity));

    lav.accRma = FlightScenario_GetAccRma();

    HostIface_PutData(HIP_MSG_LAV, (uint8_t *)&lav, sizeof(lav));
}

void _Controller_SendMon()
{
    HIP_Payload_Mon_t mon;

    memcpy(mon.throttle, g_controllerState.pwm, sizeof(mon.throttle));
    mon.vbat = g_controllerState.vbat;
    mon.ch1 = g_controllerState.ch1;

    HostIface_PutData(HIP_MSG_MON, (uint8_t *)&mon, sizeof(mon));
}

/******************************************************************************/

static void _Watchdog_Task() // TODO: enable normal watchdog
{
    while (1)
    {
        uint32_t now = xTaskGetTickCount();

        if ((now - g_controllerState.lastPing) > 2000)
        {
            g_controllerState.mState = MachineState_Disarmed;
            EC_Enable(0);
        }

        vTaskDelay(500);
    }
}

/******************************************************************************/

uint64_t Controller_GetUS()
{
    return Timer_GetRuntime(1);
}
