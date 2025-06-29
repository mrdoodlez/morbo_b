#ifndef _SCENARIOS_H_
#define _SCENARIOS_H_

#include "main.h"
#include "mhelpers.h"
#include "motion_fx.h"

#ifdef __cplusplus
extern "C"
{
#endif
    typedef enum
    {
        FlightScenario_None,
        FlightScenario_Debug,

        FlightScenario_Total,
    } FlightScenario_t;

    typedef enum
    {
        FlightScenario_Result_None,
        FlightScenario_Result_OK,
        FlightScenario_Result_Error,
    } FlightScenario_Result_t;

    typedef enum
    {
        FlightScenario_Input_Meas,
        FlightScenario_Input_DebugPwms,
    } FlightScenario_Input_t;

    typedef enum
    {
        FS_StateFlags_MeasValid = 1 << 0,
        FS_StateFlags_StateDotValid = 1 << 1,
        FS_StateFlags_StateValid = 1 << 2,
    } FS_StateFlags_t;

    typedef struct
    {
        float time;
        MFX_output_t imu;

        float a[FS_NUM_AXIS];
        float v[FS_NUM_AXIS];
        float p[FS_NUM_AXIS];

        float r[FS_NUM_AXIS];
        float dr[FS_NUM_AXIS];
        float w[FS_NUM_AXIS];

        float u[FS_NUM_AXIS];
        float thrustN[4];  // [N]

        uint32_t flags;
    } FS_State_t;

    typedef struct
    {
        float pwm[4];
    } ControlOutputs_t;

    typedef struct
    {
        struct
        {
            float kp;
            float kd;
            float ki;
        } pos;
        struct
        {
            float kp[3];
            float kd[3];
            float ki[3];
        } att;
    } FS_PID_Koeffs_t;

    int FlightScenario_SetScenario(FlightScenario_t s);

    int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data);

    int FlightScenario_Set_PID_Koeffs(FS_PID_Koeffs_t* koeffs);

    int FlightScenario_Set_Mass(float mass);

    FlightScenario_Result_t FlightScenario(ControlOutputs_t *output);

    void FlightScenario_ResetPos();

    void FlightScenario_Init(int algoFreq);

    const FS_State_t* FlightScenario_GetState();

#ifdef __cplusplus
}
#endif

#endif // _SCENARIOS_H_
