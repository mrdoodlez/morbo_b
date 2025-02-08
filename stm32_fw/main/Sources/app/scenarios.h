#ifndef _SCENARIOS_H_
#define _SCENARIOS_H_

#include "main.h"
#include "mhelpers.h"

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

    typedef struct
    {
        float pwm[4];
    } ControlOutputs_t;

    int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data);

    FlightScenario_Result_t FlightScenario(ControlOutputs_t *output);

    void FlightScenario_GetPAT(FlightScenario_PAT_t *pat);

#ifdef __cplusplus
}
#endif

#endif // _SCENARIOS_H_