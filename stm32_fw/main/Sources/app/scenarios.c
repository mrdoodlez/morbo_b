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
static FlightScenario_Result_t FlightScenarioFunc_VelSet(ControlOutputs_t *);

static const struct
{
    int encPulses;
    float wheelD;
    float wheelBase;
} g_params =
    {
        .encPulses = 16,
        .wheelD = 0.075,
        .wheelBase = 0.27,
};

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
        FlightScenarioFunc_VelSet,
};

static struct
{
    FlightScenarioDesc_t scenarios[NUM_PROGRAMS];
    uint32_t scCounter;

    FS_State_t measBuff[FS_NUM_EPOCHS];
    uint8_t epochIdx;
    uint8_t prevIdx;

    int isStatic;

    uint32_t epochCounter;

    RMA_t enc[FS_Wheel_Cnt];

    float pwm[FS_Wheel_Cnt];

    struct
    {
        float v;
        float w;
        char stop;
    } cmdVel;

    FS_PID_Koeffs_t kPid;

    struct
    {
        float v;
        float w;
    } iSum;
} _copterState;

/******************************************************************************/

void FlightScenario_Init(int algoFreq)
{
    memset(&_copterState, 0, sizeof(_copterState));

    _copterState.enc[FS_Wheel_L].windowSz = 16;
    _copterState.enc[FS_Wheel_R].windowSz = 16;

    _copterState.kPid.v.kp = 0.5;
    _copterState.kPid.v.ki = 0.05;

    _copterState.kPid.w.kp = 0.1;
    _copterState.kPid.w.ki = 0.01;

    _copterState.isStatic = 1;
}

void FlightScenario_BeginEpoch()
{
    _copterState.epochCounter++;

    _copterState.epochIdx = _copterState.epochCounter % FS_NUM_EPOCHS;
    _copterState.prevIdx = (_copterState.epochCounter + FS_NUM_EPOCHS - 1) % FS_NUM_EPOCHS;

    _copterState.measBuff[_copterState.epochIdx].flags = 0;
}

void FlightScenario_EndEpoch()
{
    // TODO; do nothing
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

int FlightScenario_SetInputs(FlightScenario_Input_t type, const void *data)
{
    if (type == FlightScenario_Input_Meas)
    {
        FS_Meas_t *meas = (FS_Meas_t *)data;

        _copterState.measBuff[_copterState.epochIdx].meas = *meas;
        _copterState.measBuff[_copterState.epochIdx].time = meas->us * 1.0e-6;

        _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_MeasValid;
    }
    else if (type == FlightScenario_Input_DebugPwms)
    {
        memcpy(_copterState.pwm, data, sizeof(_copterState.pwm));
    }
    else if (type == FlightScenario_Input_VelCmd)
    {
        struct VelCmd
        {
            float v;
            float w;
            uint32_t flags;
        };

        _copterState.cmdVel.v = ((struct VelCmd*)data)->v;
        _copterState.cmdVel.w = ((struct VelCmd*)data)->w;
        _copterState.cmdVel.stop = ((struct VelCmd*)data)->flags != 0;
    }

    return 0;
}

int FlightScenario_Set_PID_Koeffs(FS_PID_Koeffs_t *koeffs)
{
    _copterState.kPid = *koeffs;

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

static FlightScenario_Result_t Estimate()
{
    if ((_copterState.measBuff[_copterState.epochIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
    {
        if ((_copterState.measBuff[_copterState.prevIdx].flags & FS_StateFlags_MeasValid) == FS_StateFlags_MeasValid)
        {
            float dL = _copterState.measBuff[_copterState.epochIdx].meas.wheelsPulses.l - _copterState.measBuff[_copterState.prevIdx].meas.wheelsPulses.l;
            float dR = _copterState.measBuff[_copterState.epochIdx].meas.wheelsPulses.r - _copterState.measBuff[_copterState.prevIdx].meas.wheelsPulses.r;

            dL *= _copterState.pwm[FS_Wheel_L] > 0 ? 1.0 : -1.0;
            dR *= _copterState.pwm[FS_Wheel_R] > 0 ? 1.0 : -1.0;

            dL *= 2.0 * M_PI / g_params.encPulses;
            dR *= 2.0 * M_PI / g_params.encPulses;

            FS_RmaUpdate(&(_copterState.enc[FS_Wheel_L]), dL);
            FS_RmaUpdate(&(_copterState.enc[FS_Wheel_R]), dR);

            dL = _copterState.enc[FS_Wheel_L].val;
            dR = _copterState.enc[FS_Wheel_R].val;

            float dt = _copterState.measBuff[_copterState.epochIdx].time
                - _copterState.measBuff[_copterState.prevIdx].time;

            float wL = dL / dt;
            float wR = dR / dt;

            float vL = wL * g_params.wheelD / 2.0;
            float vR = wR * g_params.wheelD / 2.0;

            float vlin = (vL + vR) / 2.0;

            float omega_ = _copterState.measBuff[_copterState.prevIdx].w[2];
            float omega = (vR - vL) / g_params.wheelBase;

            float phi_ = _copterState.measBuff[_copterState.prevIdx].r[2];
            float phi = 0.0;

            FS_Integate(&phi, phi_, omega_, omega, dt);

            if (phi > 2.0 * M_PI)
                phi -= 2.0 * M_PI;
            else if (phi < 0.0)
                phi += 2.0 * M_PI;

            Vec3D_t *v_ = (Vec3D_t*)(_copterState.measBuff[_copterState.prevIdx].v);
            Vec3D_t *p_ = (Vec3D_t*)(_copterState.measBuff[_copterState.prevIdx].p);

            Vec3D_t *v = (Vec3D_t*)(_copterState.measBuff[_copterState.epochIdx].v);
            Vec3D_t *p = (Vec3D_t*)(_copterState.measBuff[_copterState.epochIdx].p);

            v->x[0] = vlin * cos(phi);
            v->x[1] = vlin * sin(phi);
            v->x[2] = 0.0;

            FS_Integate3D(p, p_, v, v_, dt);

            _copterState.measBuff[_copterState.epochIdx].w[0] = 0.0;
            _copterState.measBuff[_copterState.epochIdx].w[1] = 0.0;
            _copterState.measBuff[_copterState.epochIdx].w[2] = omega;

            _copterState.measBuff[_copterState.epochIdx].r[0] = 0.0;
            _copterState.measBuff[_copterState.epochIdx].r[1] = 0.0;
            _copterState.measBuff[_copterState.epochIdx].r[2] = phi;

            _copterState.measBuff[_copterState.epochIdx].vlin = vlin;

            _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_StateValid;
        }
    }

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_Debug(ControlOutputs_t *output)
{
    memcpy(output->pwm, _copterState.pwm, sizeof(_copterState.pwm));
    return FlightScenario_Result_OK;
}

#define PMAX    0.9

static inline float truncate( float x)
{
    if (x > PMAX)
        x = PMAX;
    else if (x < -PMAX)
        x = -PMAX;
    return x;
}

static FlightScenario_Result_t FlightScenarioFunc_VelSet(ControlOutputs_t *output)
{
    if (_copterState.cmdVel.stop)
    {
        _copterState.iSum.v = 0.0;
        _copterState.iSum.w = 0.0;

        output->pwm[FS_Wheel_R] = 0.0;
        output->pwm[FS_Wheel_L] = 0.0;
    }
    else
    {
        float el = _copterState.cmdVel.v - _copterState.measBuff[_copterState.epochIdx].vlin;
        float er = _copterState.cmdVel.w - _copterState.measBuff[_copterState.epochIdx].w[2];

        _copterState.iSum.v += el;
        _copterState.iSum.w += er;

        float ptot = _copterState.kPid.v.kp * el + _copterState.kPid.v.ki * _copterState.iSum.v;
        float pdif = _copterState.kPid.w.kp * er + _copterState.kPid.w.ki * _copterState.iSum.w;

        output->pwm[FS_Wheel_R] = truncate((ptot + pdif) / 2.0);
        output->pwm[FS_Wheel_L] = truncate((ptot - pdif) / 2.0);
    }

    _copterState.pwm[FS_Wheel_L] = output->pwm[FS_Wheel_L];
    _copterState.pwm[FS_Wheel_R] = output->pwm[FS_Wheel_R];

    return FlightScenario_Result_OK;
}
