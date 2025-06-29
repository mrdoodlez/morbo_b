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

    Butterworth2_t bwFiltE[FS_NUM_AXIS];
    Butterworth2_t bwFiltW[FS_NUM_AXIS];

    float wuYaw;

    int inAir;

    FS_PID_Koeffs_t kPid;
    float massKg;
} _copterState;

static const struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} butterCoeffsE = {
    .b0 = 0.00134871f,
    .b1 = 0.00269742f,
    .b2 = 0.00134871f,
    .a1 = -1.8934641f,
    .a2 = 0.8988590f,
},
  butterCoeffsW = {
      .b0 = 0.0913149f,
      .b1 = 0.1826298f,
      .b2 = 0.0913149f,
      .a1 = -0.9824058f,
      .a2 = 0.3476654f,
};

static uint8_t IsStatic();

extern float gravity;

/******************************************************************************/

static inline float WeightForThrust(float thrustGramms, float F0, float k)
{
    return 1.0f / (1.0f + expf(-k * (thrustGramms - F0)));
}

inline static float VoltageForThrust(float thrustGramms)
{
    float w = WeightForThrust(thrustGramms, 25, 0.1);

    extern const float motorCalibCoeffs[];
    return w * (motorCalibCoeffs[0] * thrustGramms * thrustGramms
        + motorCalibCoeffs[1] * thrustGramms + motorCalibCoeffs[2]);
}

/******************************************************************************/

void FlightScenario_Init(int algoFreq)
{
    memset(&_copterState, 0, sizeof(_copterState));

    _copterState.accRma.windowSz = 0.5 * algoFreq;
    _copterState.isStatic = 1;

    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        FS_Butterworth2_Init(&(_copterState.bwFiltE[i]), butterCoeffsE.b0,
            butterCoeffsE.b1, butterCoeffsE.b2, butterCoeffsE.a1, butterCoeffsE.a2);

        FS_Butterworth2_Init(&(_copterState.bwFiltW[i]), butterCoeffsW.b0,
            butterCoeffsW.b1, butterCoeffsW.b2, butterCoeffsW.a1, butterCoeffsW.a2);
    }
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

int FlightScenario_Set_PID_Koeffs(FS_PID_Koeffs_t *koeffs)
{
    _copterState.kPid = *koeffs;

    return 0;
}

int FlightScenario_Set_Mass(float mass)
{
    _copterState.massKg = mass;

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
    return FlightScenario_Result_OK;

    if ((_copterState.measBuff[_copterState.epochIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
    {
        uint8_t prevIdx = _copterState.epochIdx == 0 ? 1 : 0;

        // in MFX rotation is returned as yaw, pitch, roll

        // roll
        _copterState.measBuff[_copterState.epochIdx].r[0] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[2]) / 180.0 * PI;

        // pitch
        if (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] < 0)
            _copterState.measBuff[_copterState.epochIdx].r[1] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] + 180.0) / 180.0 * PI;
        else
            _copterState.measBuff[_copterState.epochIdx].r[1] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[1] - 180.0) / 180.0 * PI;

        // yaw
        _copterState.measBuff[_copterState.epochIdx].r[2] = (_copterState.measBuff[_copterState.epochIdx].imu.rotation[0] - 180.0) / 180.0 * PI; // yaw

        for (int i = 0; i < FS_NUM_AXIS; i++)
        {
            _copterState.measBuff[_copterState.epochIdx].r[i]
                = FS_Butterworth2_Update(&(_copterState.bwFiltE[i]), _copterState.measBuff[_copterState.epochIdx].r[i]);
        }

        if ((fabs(_copterState.measBuff[_copterState.epochIdx].r[0]) > ROT_THR) || (fabs(_copterState.measBuff[_copterState.epochIdx].r[1]) > ROT_THR))
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

            /*

            // TOD: fix omega computation

            Quaternion_t q1 = *((Quaternion_t *)&_copterState.measBuff[prevIdx].imu.quaternion);
            Quaternion_t q2 = *((Quaternion_t *)&_copterState.measBuff[_copterState.epochIdx].imu.quaternion);

            Quaternion_t q1_;
            FS_ConjQuat(&q1_, &q1);

            Quaternion_t dq;
            FS_QuatMul(&dq, &q1_, &q2);
            FS_NormQuat(&dq);

            float angle = 2.0f * acosf(fmaxf(-1.0f, fminf(1.0f, dq.w))); // clamp for safety
            float sin_half = sinf(angle / 2.0f);

            if (fabsf(sin_half) < 1e-6f || dt < 1e-6f)
            {
                memset(_copterState.measBuff[_copterState.epochIdx].w, 0,
                       sizeof(_copterState.measBuff[_copterState.epochIdx].w));
            }
            else
            {
                float axis_x = dq.x / sin_half;
                float axis_y = dq.y / sin_half;
                float axis_z = dq.z / sin_half;

                _copterState.measBuff[_copterState.epochIdx].w[0] = (angle / dt) * axis_x;
                _copterState.measBuff[_copterState.epochIdx].w[1] = (angle / dt) * axis_y;
                _copterState.measBuff[_copterState.epochIdx].w[2] = (angle / dt) * axis_z;
            }

            for (int i = 0; i < FS_NUM_AXIS; i++)
            {
                _copterState.measBuff[_copterState.epochIdx].w[i]
                    = FS_Butterworth2_Update(&(_copterState.bwFiltW[i]), _copterState.measBuff[_copterState.epochIdx].w[i]);
            }

            */

            for (int i = 0; i < FS_NUM_AXIS; i++)
                _copterState.measBuff[_copterState.epochIdx].dr[i]
                    = (_copterState.measBuff[_copterState.epochIdx].r[i]
                        - _copterState.measBuff[prevIdx].r[i]) / dt;
        }
    }

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *output)
{
    memcpy(output->pwm, _copterState.pwm, sizeof(_copterState.pwm));
    return FlightScenario_Result_OK;
}

static uint8_t IsStatic()
{
    return 0;

    // TODO: rewrite

    float thrH = 1.5 * _copterState.avgA;
    float thrL = 0.8 * thrH;

    if (_copterState.isStatic && (_copterState.accRma.val > thrH))
        _copterState.isStatic = 0;
    if (!_copterState.isStatic && (_copterState.accRma.val < thrL))
        _copterState.isStatic = 1;

    return _copterState.isStatic;
}
