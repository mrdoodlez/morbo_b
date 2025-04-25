#include "scenarios.h"
#include <string.h>
#include "motion_fx.h"
#include "mhelpers.h"
#include "controller.h"
#include "monitor.h"
#include <math.h>

#define FS_NUM_EPOCHS 2
#define NUM_PROGRAMS 2

#define NUM_ENGINES 4

static FlightScenario_Result_t Estimate();

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *);
static FlightScenario_Result_t FlightScenarioFunc_Windup(ControlOutputs_t *);

static FlightScenario_Result_t FlightScenario_Hover(ControlOutputs_t *output);

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
    float thrustN[4];  // [N]

    int inAir;
} _copterState;

static uint8_t IsStatic();

extern float gravity;

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

// TODO: implement preemtive scenarios!
int FlightScenario_SetScenario(FlightScenario_t s)
{
    uint32_t curr = _copterState.scCounter;
    uint32_t set = (_copterState.scenarios[curr].status != FlightScenarioStatus_Running) // || isPreemtive
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

    // if we are flying let's not fall
    if (_copterState.inAir)
        return FlightScenario_Hover(output);

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

            FS_ScaleVec(gravity, a);
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

                // in MFX rotation is returned as yaw, pitch, roll
                _copterState.pat.r[0] = _copterState.measBuff[prevIdx].imu.rotation[2]; // roll
                _copterState.pat.r[1] = _copterState.measBuff[prevIdx].imu.rotation[1]; // pitch
                _copterState.pat.r[2] = _copterState.measBuff[prevIdx].imu.rotation[0]; // yaw
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

    extern float copterMassKg;
    float deltaThrust = copterMassKg;
    float totThrust = deltaThrust * sigVal;
    float engThrust = totThrust / NUM_ENGINES;
    float motorVoltage = VoltageForThrust(engThrust * 1000.0);  // input in gramms
    float pwm = motorVoltage / Monitor_GetVbat();
    for (int i = 0; i < NUM_ENGINES; i++)
    {
        _copterState.pwm[i] = pwm;
        _copterState.thrustN[i] = engThrust * gravity;
        output->pwm[i] = pwm;
    }

    _copterState.inAir = 1;

    return FlightScenario_Result_OK;
}

static const float _thrustMatrix[] = {
    +1.0, +1.0, +1.0, +1.0,
    +1.0, +1.0, -1.0, -1.0,
    -1.0, +1.0, -1.0, +1.0,
    -1.0, +1.0, +1.0, -1.0,
};

extern void __dbg_hook(int arg);

static FlightScenario_Result_t FlightScenario_Hover(ControlOutputs_t *output)
{
    float tgRot[] = {0, 0, 0};
    float kpRot[] = {1, 1, 1};
    float inertia[] = {1, 1, 1};

    float b[NUM_ENGINES];
    extern float copterMassKg;
    b[0] = copterMassKg * gravity;
    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        b[1 + i] = inertia[i] * (kpRot[i] * (tgRot[i] - _copterState.pat.r[i]) /* + */);
    }

    float A[NUM_ENGINES * NUM_ENGINES];
    memcpy(A, _thrustMatrix, sizeof(A));

    float x[NUM_ENGINES];
    if (FS_SolveLS(NUM_ENGINES, A, b, x ) == 1)
    {
        __dbg_hook(100010);

        for (int i = 0; i < NUM_ENGINES; i++)
        {
            _copterState.thrustN[i] = x[i];
            float engThrustG = x[i] / gravity * 1000.0;
            float motorVoltage = VoltageForThrust(engThrustG);
            float pwm = motorVoltage / Monitor_GetVbat();

            _copterState.pwm[i] = pwm;
            output->pwm[i] = pwm;
        }

        return FlightScenario_Result_OK;
    }

    return FlightScenario_Result_None;
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
