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
static FlightScenario_Result_t FlightScenarioFunc_GoTo(ControlOutputs_t *);
static FlightScenario_Result_t FlightScenarioFunc_TrgTrack(ControlOutputs_t *);

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
    FlightScenarioFunc_t exec;
    uint32_t flags;
} FlightScenarioDesc_t;

typedef struct
{
    uint64_t startUs;
    FlightScenarioStatus_t status;
    FlightScenarioDesc_t desc;
} FlightScenarioCtx_t;

typedef enum
{
    FlightScenarioFlags_IsInfinite = 1 << 0,

} FlightScenarioFlags_t;

static const FlightScenarioDesc_t _descs[FlightScenario_Total] =
    {
        {0, 0},
        {FlightScenarioFunc_Debug, FlightScenarioFlags_IsInfinite},
        {FlightScenarioFunc_VelSet, FlightScenarioFlags_IsInfinite},
        {FlightScenarioFunc_GoTo, FlightScenarioFlags_IsInfinite},
        {FlightScenarioFunc_TrgTrack, FlightScenarioFlags_IsInfinite},
};

static struct
{
    FlightScenarioCtx_t scenarios[NUM_PROGRAMS];
    uint32_t scCounter;

    FS_State_t measBuff[FS_NUM_EPOCHS];
    uint8_t epochIdx;
    uint8_t prevIdx;

    int isStatic;

    uint32_t epochCounter;

    RMA_t enc[FS_Wheel_Cnt];
    RMA_t omega;

    float pwm[FS_Wheel_Cnt];

    float initPhi;

    struct
    {
        struct
        {
            float v;
            float w;
            char stop;
        } cmd;
    } velController;

    struct
    {
        struct
        {
            float x;
            float y;
            float phi;
            char phiSet;
        } cmd;

        uint64_t replanUs;

        char newInput;
    } posController;

    struct
    {
        float dx;
        float dy;
        float tdx;
        float tdy;

        uint64_t lastTrgPosUs;
    } trgTrackController;

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

    _copterState.omega.windowSz = 16;

    _copterState.kPid.v.kp = 0.5;
    _copterState.kPid.v.ki = 0.05;

    _copterState.kPid.w.kp = 0.1;
    _copterState.kPid.w.ki = 0.01;

    _copterState.isStatic = 1;

    _copterState.initPhi = 100500;
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

int FlightScenario_SetScenario(FlightScenario_t s)
{
    uint32_t curr = _copterState.scCounter;
    uint32_t set = ((_copterState.scenarios[curr].status != FlightScenarioStatus_Running) || ((_copterState.scenarios[curr].desc.flags & FlightScenarioFlags_IsInfinite) != 0))
                       ? curr
                       : (curr + 1) % NUM_PROGRAMS;

    _copterState.scenarios[set].status = FlightScenarioStatus_Pending;
    _copterState.scenarios[set].desc = _descs[s];

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
        _copterState.velController.cmd.v = ((HIP_Payload_SetVels_t *)data)->v;
        _copterState.velController.cmd.w = ((HIP_Payload_SetVels_t *)data)->w;
        _copterState.velController.cmd.stop = ((HIP_Payload_SetVels_t *)data)->flags != 0;
    }
    else if (type == FlightScenario_Input_PosCmd)
    {
        HIP_Payload_SetPos_t *cmd = (HIP_Payload_SetPos_t *)data;

        if ((cmd->flags & HIP_SetPos_Flags_IsRelative) != 0)
        {
            float c = cosf(_copterState.measBuff[_copterState.epochIdx].r[2]);
            float s = sinf(_copterState.measBuff[_copterState.epochIdx].r[2]);
            _copterState.posController.cmd.x
                = _copterState.measBuff[_copterState.epochIdx].p[0] + c * cmd->x - s * cmd->y;
            _copterState.posController.cmd.y
                = _copterState.measBuff[_copterState.epochIdx].p[1] + s * cmd->x + c * cmd->y;
            _copterState.posController.cmd.phi
                = _copterState.measBuff[_copterState.epochIdx].r[2] + cmd->phi;
        }
        else
        {
            _copterState.posController.cmd.x = cmd->x;
            _copterState.posController.cmd.y = cmd->y;
            _copterState.posController.cmd.phi = cmd->phi;
        }

        _copterState.posController.cmd.phiSet = cmd->flags & HIP_SetPos_Flags_PhiSet;

        _copterState.posController.newInput = 1;
    }
    else if (type == FlightScenario_Input_TrgPosCmd)
    {
        HIP_Payload_TrgPos_t *cmd = (HIP_Payload_TrgPos_t *)data;

        _copterState.trgTrackController.dx = cmd->dx;
        _copterState.trgTrackController.dy = cmd->dy;

        _copterState.trgTrackController.tdx = cmd->tdx;
        _copterState.trgTrackController.tdy = cmd->tdy;

        _copterState.trgTrackController.lastTrgPosUs = Controller_GetUS();
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
        res = _copterState.scenarios[curr].desc.exec(output);

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

            float dt = _copterState.measBuff[_copterState.epochIdx].time - _copterState.measBuff[_copterState.prevIdx].time;

            float wL = dL / dt;
            float wR = dR / dt;

            float vL = wL * g_params.wheelD / 2.0;
            float vR = wR * g_params.wheelD / 2.0;

            float vlin = (vL + vR) / 2.0;

            float phi_ = _copterState.measBuff[_copterState.prevIdx].r[2];
            float phi = _copterState.measBuff[_copterState.epochIdx].meas.imu.rotation[0];

            if (fabs(_copterState.initPhi) > 365)
                _copterState.initPhi = phi;

            phi -= _copterState.initPhi;

            phi = -phi;
            phi /= 360.0;
            phi *= 2.0 * M_PI;

            float dphi = phi - phi_;
            while (dphi > M_PI)
                dphi -= 2.0 * M_PI;
            while (dphi < -M_PI)
                dphi += 2.0 * M_PI;

            float omega = dphi / dt;

            FS_RmaUpdate(&_copterState.omega, omega);
            omega = _copterState.omega.val;

            Vec3D_t *v_ = (Vec3D_t *)(_copterState.measBuff[_copterState.prevIdx].v);
            Vec3D_t *p_ = (Vec3D_t *)(_copterState.measBuff[_copterState.prevIdx].p);

            Vec3D_t *v = (Vec3D_t *)(_copterState.measBuff[_copterState.epochIdx].v);
            Vec3D_t *p = (Vec3D_t *)(_copterState.measBuff[_copterState.epochIdx].p);

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

#define PMAX 0.9

static inline float truncate(float x)
{
    if (x > PMAX)
        x = PMAX;
    else if (x < -PMAX)
        x = -PMAX;
    return x;
}

static FlightScenario_Result_t FlightScenarioFunc_VelSet(ControlOutputs_t *output)
{
    if (_copterState.velController.cmd.stop)
    {
        _copterState.iSum.v = 0.0;
        _copterState.iSum.w = 0.0;

        output->pwm[FS_Wheel_R] = 0.0;
        output->pwm[FS_Wheel_L] = 0.0;
    }
    else
    {
        float el = _copterState.velController.cmd.v - _copterState.measBuff[_copterState.epochIdx].vlin;
        float er = _copterState.velController.cmd.w - _copterState.measBuff[_copterState.epochIdx].w[2];

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

#define EPS_X 0.2
#define EPS_PHI 0.087
#define REPLAN_DT 0.2e6
#define V_MAX 0.3
#define W_MAX 1.57

static FlightScenario_Result_t FlightScenarioFunc_GoTo(ControlOutputs_t *output)
{
    uint64_t currUs = Controller_GetUS();

    float phi0 = _copterState.measBuff[_copterState.epochIdx].r[2];

    float dx = _copterState.posController.cmd.x - _copterState.measBuff[_copterState.epochIdx].p[0];
    float dy = _copterState.posController.cmd.y - _copterState.measBuff[_copterState.epochIdx].p[1];
    float dphi = _copterState.posController.cmd.phiSet ? (_copterState.posController.cmd.phi - phi0) : 0.0;

    while (dphi > M_PI)
        dphi -= 2.0 * M_PI;
    while (dphi < -M_PI)
        dphi += 2.0 * M_PI;

    if (((fabs(dx) + fabs(dy)) < EPS_X) && (fabs(dphi) < EPS_PHI))
    {
        _copterState.iSum.v = 0.0;
        _copterState.iSum.w = 0.0;

        output->pwm[FS_Wheel_R] = 0.0;
        output->pwm[FS_Wheel_L] = 0.0;
    }
    else
    {
        if (_copterState.posController.newInput || ((currUs - _copterState.posController.replanUs) > REPLAN_DT))
        {
            float c0 = cos(phi0);
            float s0 = sin(phi0);
            float dxb = c0 * dx + s0 * dy;
            float dyb = -s0 * dx + c0 * dy;

            if (fabs(dyb) < EPS_X / 2)
            {
                if (fabs(dxb) < EPS_X / 2) // pure rotation
                {
                    float dT = fabs(dphi) / W_MAX;
                    _copterState.velController.cmd.v = 0;
                    _copterState.velController.cmd.w = fabs(dphi) > EPS_PHI ? dphi / dT : 0.0;
                }
                else // straight line case
                {
                    float arcLen = fabs(dxb);
                    float dT = arcLen / V_MAX;

                    _copterState.velController.cmd.v = dxb / dT;
                    _copterState.velController.cmd.w = 0;
                }
            }
            else
            {
                float theta = 2.0f * atan2f(dyb, dxb);
                if (theta > M_PI)
                    theta -= 2.0f * M_PI;
                if (theta < -M_PI)
                    theta += 2.0f * M_PI;

                float s = sin(theta);
                float c = cos(theta);
                float denom1 = (1.0f - c);

                float k;
                if (fabs(denom1) > 1.0e-6f)
                {
                    k = dyb / denom1;
                }
                else if (fabs(s) > 1.0e-6f)
                {
                    k = dxb / s;
                }
                else
                {
                    return FlightScenario_Result_Error;
                }

                float arcLen = fabs(dxb) + fabs(dyb);
                float dT = arcLen / V_MAX;

                _copterState.velController.cmd.w = theta / dT;
                _copterState.velController.cmd.v = k * _copterState.velController.cmd.w;

                if (_copterState.velController.cmd.v > V_MAX)
                    _copterState.velController.cmd.v = V_MAX;
                else if (_copterState.velController.cmd.v < -V_MAX)
                    _copterState.velController.cmd.v = -V_MAX;
            }

            _copterState.posController.replanUs = Controller_GetUS();
            _copterState.posController.newInput = 0;
        }

        return FlightScenarioFunc_VelSet(output);
    }

    return FlightScenario_Result_OK;
}

#define TRG_TRK_LOST_DT     1.2e6
#define TRG_TRK_KV          0.6f
#define TRG_TRK_KW          0.4f
#define TRG_TRK_EPS_X       0.05f
#define TRG_TRK_EPS_Y       0.05f

static FlightScenario_Result_t FlightScenarioFunc_TrgTrack(ControlOutputs_t *output)
{
    uint64_t currUs = Controller_GetUS();

    if ((currUs - _copterState.trgTrackController.lastTrgPosUs) > TRG_TRK_LOST_DT)
    {
        _copterState.iSum.v = 0.0f;
        _copterState.iSum.w = 0.0f;

        _copterState.velController.cmd.v = 0.0f;
        _copterState.velController.cmd.w = 0.0f;

        output->pwm[FS_Wheel_R] = 0.0f;
        output->pwm[FS_Wheel_L] = 0.0f;

        return FlightScenario_Result_OK;
    }

    float ex = _copterState.trgTrackController.dx  - _copterState.trgTrackController.tdx;
    float ey = _copterState.trgTrackController.dy  - _copterState.trgTrackController.tdy;

    if (fabsf(ex) < TRG_TRK_EPS_X) ex = 0.0f;
    if (fabsf(ey) < TRG_TRK_EPS_Y) ey = 0.0f;

    // -------- P control: position error -> v, w --------
    // X forward: ex>0  -> move forward
    // Y left:    ey>0  -> rotate CCW (positive w) so target moves toward center
    float v_cmd = TRG_TRK_KV * ex;
    float w_cmd = TRG_TRK_KW * ey;

    if (v_cmd >  V_MAX) v_cmd =  V_MAX;
    if (v_cmd < -V_MAX) v_cmd = -V_MAX;

    if (w_cmd >  W_MAX) w_cmd =  W_MAX;
    if (w_cmd < -W_MAX) w_cmd = -W_MAX;

    _copterState.velController.cmd.v = v_cmd;
    _copterState.velController.cmd.w = w_cmd;

    return FlightScenarioFunc_VelSet(output);
}
