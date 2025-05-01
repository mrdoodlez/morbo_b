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

#ifndef PI
#define PI 3.14159265358979323846
#endif

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

    RMA_t accRma;
    float avgA;

    int isStatic;

    uint32_t epochCounter;

    float pwm[4];

    float S[FS_NUM_AXIS];

    float initYaw;

    int inAir;
} _copterState;

static uint8_t IsStatic();

extern float gravity;

/******************************************************************************/

inline static float VoltageForThrust(float thrustGramms)
{
    extern const float motorCalibCoeffs[];
    return motorCalibCoeffs[0] * thrustGramms * thrustGramms + motorCalibCoeffs[1] * thrustGramms + motorCalibCoeffs[2];
}

/******************************************************************************/

void FlightScenario_Init(int algoFreq)
{
    memset(&_copterState, 0, sizeof(_copterState));

    _copterState.accRma.windowSz = 0.5 * algoFreq;
    _copterState.initYaw = 100500;
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

    FlightScenario_Result_t res = FlightScenario_Result_None;

    if (_copterState.scenarios[curr].status == FlightScenarioStatus_Running)
        res = _copterState.scenarios[curr].exec(output);
    // if we are flying let's not fall
    else if (_copterState.inAir)
        res = FlightScenario_Hover(output);

    _copterState.epochIdx = (_copterState.epochIdx + 1) % FS_NUM_EPOCHS;
    _copterState.measBuff[_copterState.epochIdx].flags = 0;

    _copterState.epochCounter++;

    return res;
}

void FlightScenario_ResetPos()
{
    for (int i = 0; i < FS_NUM_EPOCHS; i++)
        FS_ZeroVec((Vec3D_t *)&(_copterState.measBuff[i].p));
}

const FS_State_t *FlightScenario_GetState()
{
    return &(_copterState.measBuff[_copterState.epochIdx]);
}

/******************************************************************************/

#define ROT_THR ((30.0 / 180.0) * PI)

static FlightScenario_Result_t Estimate()
{
    if ((_copterState.measBuff[_copterState.epochIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
    {
        uint8_t prevIdx = _copterState.epochIdx == 0 ? 1 : 0;

        // in MFX rotation is returned as yaw, pitch, roll

        if (_copterState.initYaw > 360)
            _copterState.initYaw = _copterState.measBuff[_copterState.epochIdx].imu.rotation[0];

        _copterState.measBuff[_copterState.epochIdx].r[0] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[2]) / 180.0 * PI; // roll

        if (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] < 0)
            _copterState.measBuff[_copterState.epochIdx].r[1] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] + 180.0) / 180.0 * PI;
        else
            _copterState.measBuff[_copterState.epochIdx].r[1] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] - 180.0) / 180.0 * PI;

        _copterState.measBuff[_copterState.epochIdx].r[2] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[0] - _copterState.initYaw) / 180.0 * PI; // yaw

        if ((fabs(_copterState.measBuff[_copterState.epochIdx].r[0]) > ROT_THR)
            || (fabs(_copterState.measBuff[_copterState.epochIdx].r[1]) > ROT_THR))
        {
            return FlightScenario_Result_Error;
        }

        Vec3D_t *a = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].a;
        *a = *(Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].imu.linear_acceleration;

        FS_ScaleVec(gravity, a);
        FS_VecRotQuat(a, (Quaternion_t *)&_copterState.measBuff[_copterState.epochIdx].imu.quaternion);

        float ma = FS_NormVec(a);
        FS_RmaUpdate(&_copterState.accRma, ma);
        _copterState.avgA += (ma - _copterState.avgA) / (_copterState.epochCounter + 1);

        if ((_copterState.measBuff[prevIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
        {
            Vec3D_t *a_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].a;

            float dt = _copterState.measBuff[_copterState.epochIdx].time - _copterState.measBuff[prevIdx].time;

            Vec3D_t *v = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].v;
            Vec3D_t *v_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].v;

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
            }
        }
    }

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *output)
{
    memcpy(output->pwm, _copterState.pwm, sizeof(_copterState.pwm));
    return FlightScenario_Result_OK;
}

extern void __dbg_hook(int arg);

static FlightScenario_Result_t FlightScenarioFunc_Windup(ControlOutputs_t *output)
{
    static const float LEN_S = 1.0;
    uint32_t curr = _copterState.scCounter;
    float runTime = (Controller_GetUS() - _copterState.scenarios[curr].startUs) * 1e-6;

    if (runTime > LEN_S)
    {
        _copterState.scenarios[curr].status = FlightScenarioStatus_Finished;

        __dbg_hook(100010);

        return FlightScenario_Result_None;
    }

    float ratio = runTime / LEN_S;
    float sigVal, sigValDot, sigValDotDot;
    FS_Sigmoid(ratio, &sigVal, &sigValDot, &sigValDotDot);

    extern float copterMassKg;
    float deltaThrust = copterMassKg;
    float totThrust = deltaThrust * sigVal;
    float engThrust = totThrust / NUM_ENGINES;
    float motorVoltage = VoltageForThrust(engThrust * 1000.0); // input in gramms
    float pwm = motorVoltage / Monitor_GetVbat();
    for (int i = 0; i < NUM_ENGINES; i++)
    {
        _copterState.measBuff[_copterState.epochIdx].thrustN[i] = engThrust * gravity;
        _copterState.pwm[i] = pwm;
        output->pwm[i] = pwm;
    }

    _copterState.inAir = 1;

    return FlightScenario_Result_OK;
}

static const float _thrustMatrix[] = {
    +1.0,
    +1.0,
    +1.0,
    +1.0,

    +1.0,
    +1.0,
    -1.0,
    -1.0,

    -1.0,
    +1.0,
    -1.0,
    +1.0,

    -1.0,
    +1.0,
    +1.0,
    -1.0,
};

#define KP 0.8f
#define KI 0.02f

static FlightScenario_Result_t FlightScenario_Hover(ControlOutputs_t *output)
{
    float tgRot[] = {0, 0, 0};
    float kpRot[] = {KP, KP, 0.0};
    float kiRot[] = {KI, KI, 0.0};

    float b[NUM_ENGINES];
    extern float copterMassKg;
    b[0] = copterMassKg * gravity;
    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        _copterState.measBuff[_copterState.epochIdx].e[i] = tgRot[i] - _copterState.measBuff[_copterState.epochIdx].r[i];
        _copterState.S[i] += _copterState.measBuff[_copterState.epochIdx].e[i];
        b[1 + i] = kpRot[i] * _copterState.measBuff[_copterState.epochIdx].e[i] + kiRot[i] * _copterState.S[i]; /* + */
    }

    float A[NUM_ENGINES * NUM_ENGINES];
    memcpy(A, _thrustMatrix, sizeof(A));

    float x[NUM_ENGINES];
    if (FS_SolveLS(NUM_ENGINES, A, b, x) == 1)
    {
        for (int i = 0; i < NUM_ENGINES; i++)
        {
            _copterState.measBuff[_copterState.epochIdx].thrustN[i] = x[i];
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
