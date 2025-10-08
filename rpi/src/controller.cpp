#include "controller.h"
#include "comm.h"
#include "host_interface_cmds.h"
#include "host_interface.h"

#include <iostream>

static int _SetWorkMode(uint8_t iMode, uint8_t fcMode);
static int _EnableMessage(uint16_t msgId, uint16_t periodMs);

static int _RoverConfig();

enum  IMU_Mode_t
{
    IMU_Mode_Idle,      /*0*/
    IMU_Mode_CalAcc,    /*1*/
    IMU_Mode_CalGyro,   /*2*/
    IMU_Mode_CalMag,    /*3*/
    IMU_Mode_Fusion,    /*4*/
};

enum  FlightScenario_t
{
    FlightScenario_None,
    FlightScenario_Debug,
    FlightScenario_VelSet,
    FlightScenario_GoTo,

    FlightScenario_Total,
};

int Controller_Start()
{
    Comm_Start();

    int rc = _RoverConfig();
    if (rc)
    {
        std::cout << "Rover config faild, rc: " << rc << std::endl;
        return -10;
    }

    return 0;
}

static int _RoverConfig()
{
    if (_SetWorkMode(IMU_Mode_Fusion, FlightScenario_GoTo))
        return -10;

    if (_EnableMessage(HIP_MSG_PVT, 100))
        return -20;

    if (_EnableMessage(HIP_MSG_WHT, 100))
        return -30;

    return 0;
}

/*******************************************************************************/

static int _SendCommand(uint16_t id, const uint8_t *buff, size_t len)
{
    HostIface_PutData(id, buff, len);
    HostIface_Send();

    bool isAck;
    int rc = WaitForAck(id, std::chrono::milliseconds(500), isAck);

    if ((rc == 0) && isAck)
        return 0;

    std::cout << "command " << id << " not acked" << std::endl;

    return -10;
}

static int _SetWorkMode(uint8_t iMode, uint8_t fcMode)
{
    HIP_Payload_WM_t wm;
    wm.fcMode = fcMode;
    wm.imuMode = iMode;

    return _SendCommand(HIP_MSG_WM, (uint8_t *)&wm, sizeof(wm));
}

static int _EnableMessage(uint16_t msgId, uint16_t periodMs)
{
    HIP_Payload_EM_t em;

    em.msgId = msgId;
    em.msgPeriod = periodMs / 100;

    return _SendCommand(HIP_MSG_EM, (uint8_t *)&em, sizeof(em));
}