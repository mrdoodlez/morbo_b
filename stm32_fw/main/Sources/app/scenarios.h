#ifndef _SCENARIOS_H_
#define _SCENARIOS_H_

#include "main.h"
#include "mhelpers.h"
#include "motion_fx.h"

#define GRAVITY 9.806

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

    typedef struct
    {
        float p[FS_NUM_AXIS];
        float r[FS_NUM_AXIS];
        float time;
    } FlightScenario_PAT_t;

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

        float a[3];
        float v[3];
        float p[3];

        uint32_t flags;
    } FS_State_t;

    typedef struct
    {
        float pwm[4];
    } ControlOutputs_t;

    int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data);

    FlightScenario_Result_t FlightScenario(ControlOutputs_t *output);

    void FlightScenario_GetPAT(FlightScenario_PAT_t *pat);

    void FlightScenario_ResetPos();

    void FlightScenario_Init(int algoFreq);

    float FlightScenario_GetAccRma();

    void FlightScenario_GetState(FS_State_t *s);

#ifdef __cplusplus
}
#endif

#endif // _SCENARIOS_H_