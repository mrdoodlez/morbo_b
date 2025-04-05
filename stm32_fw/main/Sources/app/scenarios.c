#include "scenarios.h"
#include <string.h>
#include "motion_fx.h"
#include "mhelpers.h"
#include <math.h>

#define FS_NUM_EPOCHS 2
static struct
{
    FlightScenario_t scenario;

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

void FlightScenario_Init(int algoFreq)
{
    memset(&_copterState, 0, sizeof(_copterState));

    _copterState.accRma.windowSz = 0.5 * algoFreq;
    _copterState.isStatic = 1;

    _copterState.scenario = FlightScenario_None;
}

int FlightScenario_SetScenario(FlightScenario_t s)
{
    _copterState.scenario = s;
    return 0;
}

int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data)
{
    if (type == FlightScenario_Input_Meas)
    {
        struct
        {
            uint64_t time;
            MFX_output_t meas;
        } meas;

        memcpy((void *)&meas, data, sizeof(meas));

        _copterState.measBuff[_copterState.epochIdx].imu = meas.meas;
        _copterState.measBuff[_copterState.epochIdx].time = meas.time * 1.0e-6;

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

    if (_copterState.scenario == FlightScenario_Debug)
    {
        memcpy(output->pwm, _copterState.pwm, sizeof(_copterState.pwm));
        return FlightScenario_Result_OK;
    }

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

void FlightScenario_GetState(FS_State_t* s)
{
    *s = _copterState.measBuff[_copterState.epochIdx];
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
