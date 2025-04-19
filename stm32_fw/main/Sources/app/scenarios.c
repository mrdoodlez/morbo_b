#include "scenarios.h"
#include <string.h>
#include "motion_fx.h"
#include "mhelpers.h"
#include "controller.h"
#include "monitor.h"
#include <math.h>

#define FS_NUM_EPOCHS 2
#define NUM_PROGRAMS 2

static FlightScenario_Result_t Estimate();

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *);
static FlightScenario_Result_t FlightScenarioFunc_Windup(ControlOutputs_t *);

typedef enum
{
    FlightScenarioStatus_Finished,
    FlightScenarioStatus_Pending,
    FlightScenarioStatus_Running
} FlightScenarioStatus_t;

typedef FlightScenario_Result_t (*FlightScenarioFunc_t)(ControlOutputs_t *output);

typedef struct
{
    uint64_t startUs;
    FlightScenarioStatus_t status;
    FlightScenarioFunc_t exec;
    FlightScenario_t type;
} FlightScenarioDesc_t;

FlightScenarioFunc_t _execs[FlightScenario_Total] =
    {
        NULL,
        FlightScenarioFunc_Debug,
        FlightScenarioFunc_Windup,
};

static struct
{
    FlightScenarioDesc_t scenarios[NUM_PROGRAMS];
    uint32_t scCounter;

    FS_State_t measBuff[FS_NUM_EPOCHS];
    uint8_t epochIdx;

    FlightScenario_PAT_t pat;
    RMA_t accRma;
    float avgA;

    int isStatic;

    uint32_t epochCounter;

    float pwm[4];
} _copterState;

static uint8_t IsStatic();

/******************************************************************************/

inline static float VoltageForThrust(float thrustGramms)
{
    extern const float motorCalibCoeffs[];
    return motorCalibCoeffs[0] * thrustGramms * thrustGramms
        + motorCalibCoeffs[1] * thrustGramms
        + motorCalibCoeffs[2];
}

/******************************************************************************/

void FlightScenario_Init(int algoFreq)
{
    memset(&_copterState, 0, sizeof(_copterState));

    _copterState.accRma.windowSz = 0.5 * algoFreq;
    _copterState.isStatic = 1;
}

int FlightScenario_SetScenario(FlightScenario_t s)
{
    uint32_t curr = _copterState.scCounter;
    uint32_t set = (_copterState.scenarios[curr].status != FlightScenarioStatus_Running)
                       ? curr
                       : (curr + 1) % NUM_PROGRAMS;

    _copterState.scenarios[set].type = s;
    _copterState.scenarios[set].status = FlightScenarioStatus_Pending;
    _copterState.scenarios[set].exec = _execs[s];

    return 0;
}

int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data)
{
    if (type == FlightScenario_Input_Meas)
    {
        struct
        {
            uint64_t usec;
            MFX_output_t meas;
        } meas;

        memcpy((void *)&meas, data, sizeof(meas));

        _copterState.measBuff[_copterState.epochIdx].imu = meas.meas;
        _copterState.measBuff[_copterState.epochIdx].time = meas.usec * 1.0e-6;

        _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_MeasValid;
    }
    else if (type == FlightScenario_Input_DebugPwms)
    {
        memcpy(_copterState.pwm, data, sizeof(_copterState.pwm));
    }
    return 0;
}

FlightScenario_Result_t FlightScenario(ControlOutputs_t *output)
{
    if (Estimate() == FlightScenario_Result_Error)
    {
        return FlightScenario_Result_Error;
    }

    uint32_t curr = _copterState.scCounter;
    if (_copterState.scenarios[curr].status != FlightScenarioStatus_Running)
    {
        // look for pending scenarios
        for (int i = 0; i < NUM_PROGRAMS; i++)
        {
            if (_copterState.scenarios[curr].status == FlightScenarioStatus_Pending)
            {
                _copterState.scenarios[curr].startUs = Controller_GetUS();
                _copterState.scenarios[curr].status = FlightScenarioStatus_Running;
                _copterState.scCounter = curr;
                break;
            }
            curr = (curr + 1) % NUM_PROGRAMS;
        }
    }

    if (_copterState.scenarios[curr].status == FlightScenarioStatus_Running)
        return _copterState.scenarios[curr].exec(output);

    return FlightScenario_Result_None;
}

void FlightScenario_GetPAT(FlightScenario_PAT_t *pat)
{
    memcpy(pat, &_copterState.pat, sizeof(FlightScenario_PAT_t));
}

void FlightScenario_ResetPos()
{
    for (int i = 0; i < FS_NUM_EPOCHS; i++)
        FS_ZeroVec((Vec3D_t *)&(_copterState.measBuff[i].p));
}

float FlightScenario_GetAccRma()
{
    return _copterState.accRma.val;
}

void FlightScenario_GetState(FS_State_t *s)
{
    *s = _copterState.measBuff[_copterState.epochIdx];
}

/******************************************************************************/

static FlightScenario_Result_t Estimate()
{
    if ((_copterState.measBuff[_copterState.epochIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
    {
        uint8_t prevIdx = _copterState.epochIdx == 0 ? 1 : 0;

        if ((_copterState.measBuff[prevIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
        {
            Vec3D_t *a = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].a;
            *a = *(Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].imu.linear_acceleration;

            FS_ScaleVec(GRAVITY, a);
            FS_VecRotQuat(a, (Quaternion_t *)&_copterState.measBuff[_copterState.epochIdx].imu.quaternion);

            Vec3D_t *a_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].a;

            float dt = _copterState.measBuff[_copterState.epochIdx].time - _copterState.measBuff[prevIdx].time;

            Vec3D_t *v = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].v;
            Vec3D_t *v_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].v;

            float ma = FS_NormVec(a);
            FS_RmaUpdate(&_copterState.accRma, ma);
            _copterState.avgA += (ma - _copterState.avgA) / (_copterState.epochCounter + 1);

            if (!IsStatic())
                FS_Integate3D(v, v_, a, a_, dt);
            else
                FS_ZeroVec(v);

            _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_StateDotValid;

            if ((_copterState.measBuff[prevIdx].flags & FS_StateFlags_StateDotValid) == FS_StateFlags_StateDotValid)
            {
                Vec3D_t *p = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].p;
                Vec3D_t *p_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].p;

                FS_Integate3D(p, p_, v, v_, dt);

                _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_StateValid;

                memcpy(_copterState.pat.p, p, sizeof(_copterState.pat.p));
                memcpy(_copterState.pat.r, _copterState.measBuff[prevIdx].imu.rotation, sizeof(_copterState.pat.r));
                _copterState.pat.time = _copterState.measBuff[_copterState.epochIdx].time;
            }
        }
    }

    _copterState.epochIdx = (_copterState.epochIdx + 1) % FS_NUM_EPOCHS;
    _copterState.measBuff[_copterState.epochIdx].flags = 0;

    _copterState.epochCounter++;

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *output)
{
    memcpy(output->pwm, _copterState.pwm, sizeof(_copterState.pwm));
    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_Windup(ControlOutputs_t *output)
{
    static const float LEN_S = 5.0;
    uint32_t curr = _copterState.scCounter;
    float runTime = (Controller_GetUS() - _copterState.scenarios[curr].startUs) * 1e-6;

    if (runTime > LEN_S)
    {
        _copterState.scenarios[curr].status = FlightScenarioStatus_Finished;
        return FlightScenario_Result_None;
    }

    float ratio = runTime / LEN_S;
    float sigVal, sigValDot, sigValDotDot;
    FS_Sigmoid(ratio, &sigVal, &sigValDot, &sigValDotDot);

    extern float copterMassG;
    float deltaThrust = copterMassG;
    float totThrust = deltaThrust * sigVal;
    float engThrust = totThrust / 4.0;
    float motorVoltage = VoltageForThrust(engThrust);

    float pwm = motorVoltage / Monitor_GetVbat();
    for (int i = 0; i < 4; i++)
        output->pwm[i] = pwm;

    return FlightScenario_Result_OK;
}

static uint8_t IsStatic()
{
    float thrH = 1.5 * _copterState.avgA;
    float thrL = 0.8 * thrH;

    if (_copterState.isStatic && (_copterState.accRma.val > thrH))
        _copterState.isStatic = 0;
    if (!_copterState.isStatic && (_copterState.accRma.val < thrL))
        _copterState.isStatic = 1;

    return _copterState.isStatic;
}
