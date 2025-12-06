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

struct
{
    uint8_t valid;

    /* Rover geometry */
    uint16_t enc_pulses; // [pulses / rev]
    float wheel_d;       // [m]
    float wheel_base;    // [m]

    /* Averaging window */
    uint8_t omega_window_sz;

    /* PID (linear velocity) */
    float pid_v_kp;
    float pid_v_ki;
    float pid_v_kd;

    /* PID (angular velocity) */
    float pid_w_kp;
    float pid_w_ki;
    float pid_w_kd;

    /* Limits */
    float pwm_max; // [0..1]

    /* Path-planning */
    float path_eps_x;           // [m]
    float path_eps_phi;         // [rad]
    uint64_t path_replan_dt_us; // [us]
    float path_v_max;           // [m/s]
    float path_w_max;           // [rad/s]

    /* Tag tracking */
    uint64_t tag_lost_dt_us; // [us]
    float tag_kv;
    float tag_kw;
    float tag_eps_x; // [m]
    float tag_eps_y; // [m]
} g_params =
    {
        .valid = 0,
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
    uint32_t set = ((_copterState.scenarios[curr].status != FlightScenarioStatus_Running)
        || ((_copterState.scenarios[curr].desc.flags & FlightScenarioFlags_IsInfinite) != 0))
                       ? curr
                       : (curr + 1) % NUM_PROGRAMS;

    _copterState.scenarios[set].status = FlightScenarioStatus_Pending;
    _copterState.scenarios[set].desc = _descs[s];

    return 0;
}

static inline float decode_q1k(int16_t q)
{
    return (float)q / MCU_Q_SCALE_1K;
}

static inline float decode_pwm01(uint8_t q)
{
    return (float)q / 255.0f;
}

static inline float mm_to_m(uint16_t mm)
{
    return (float)mm / 1000.0f;
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
    else if (type == FlightScenario_Input_Params)
    {
        McuParams_t *src = (McuParams_t *)data;

        g_params.enc_pulses = src->enc_pulses;
        g_params.wheel_d = mm_to_m(src->wheel_d_mm);
        g_params.wheel_base = mm_to_m(src->wheel_base_mm);

        g_params.omega_window_sz = src->omega_window_sz;

        g_params.pid_v_kp = decode_q1k(src->pid_v_kp_q);
        g_params.pid_v_ki = decode_q1k(src->pid_v_ki_q);
        g_params.pid_v_kd = decode_q1k(src->pid_v_kd_q);

        g_params.pid_w_kp = decode_q1k(src->pid_w_kp_q);
        g_params.pid_w_ki = decode_q1k(src->pid_w_ki_q);
        g_params.pid_w_kd = decode_q1k(src->pid_w_kd_q);

        g_params.pwm_max = decode_pwm01(src->pwm_max_q);

        g_params.path_eps_x = decode_q1k(src->path_eps_x_q);
        g_params.path_eps_phi = decode_q1k(src->path_eps_phi_q);
        g_params.path_replan_dt_us = src->path_replan_dt_us;
        g_params.path_v_max = decode_q1k(src->path_v_max_q);
        g_params.path_w_max = decode_q1k(src->path_w_max_q);

        g_params.tag_lost_dt_us = src->tag_lost_dt_us;
        g_params.tag_kv = decode_q1k(src->tag_kv_q);
        g_params.tag_kw = decode_q1k(src->tag_kw_q);
        g_params.tag_eps_x = decode_q1k(src->tag_eps_x_q);
        g_params.tag_eps_y = decode_q1k(src->tag_eps_y_q);

        // TODO: probably we need different windows

        _copterState.enc[FS_Wheel_L].windowSz = g_params.omega_window_sz;
        _copterState.enc[FS_Wheel_R].windowSz = g_params.omega_window_sz;
        _copterState.omega.windowSz = g_params.omega_window_sz;

        g_params.valid = 1;
    }

    return 0;
}

FlightScenario_Result_t FlightScenario(ControlOutputs_t *output)
{
    if (!g_params.valid)
    {
        return FlightScenario_Result_None;
    }

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
            float dL = _copterState.measBuff[_copterState.epochIdx].meas.wheelsPulses.l
                - _copterState.measBuff[_copterState.prevIdx].meas.wheelsPulses.l;
            float dR = _copterState.measBuff[_copterState.epochIdx].meas.wheelsPulses.r
                - _copterState.measBuff[_copterState.prevIdx].meas.wheelsPulses.r;

            dL *= _copterState.pwm[FS_Wheel_L] > 0 ? 1.0 : -1.0;
            dR *= _copterState.pwm[FS_Wheel_R] > 0 ? 1.0 : -1.0;

            dL *= 2.0 * M_PI / g_params.enc_pulses;
            dR *= 2.0 * M_PI / g_params.enc_pulses;

            FS_RmaUpdate(&(_copterState.enc[FS_Wheel_L]), dL);
            FS_RmaUpdate(&(_copterState.enc[FS_Wheel_R]), dR);

            dL = _copterState.enc[FS_Wheel_L].val;
            dR = _copterState.enc[FS_Wheel_R].val;

            float dt = _copterState.measBuff[_copterState.epochIdx].time
                - _copterState.measBuff[_copterState.prevIdx].time;

            float wL = dL / dt;
            float wR = dR / dt;

            float vL = wL * g_params.wheel_d / 2.0;
            float vR = wR * g_params.wheel_d / 2.0;

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

static inline float truncate(float x)
{
    if (x > g_params.pwm_max)
        x = g_params.pwm_max;
    else if (x < -g_params.pwm_max)
        x = -g_params.pwm_max;
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
        float el = _copterState.velController.cmd.v
            - _copterState.measBuff[_copterState.epochIdx].vlin;
        float er = _copterState.velController.cmd.w
            - _copterState.measBuff[_copterState.epochIdx].w[2];

        _copterState.iSum.v += el;
        _copterState.iSum.w += er;

        float ptot = g_params.pid_v_kp * el + g_params.pid_v_ki * _copterState.iSum.v;
        float pdif = g_params.pid_w_kp * er + g_params.pid_w_ki * _copterState.iSum.w;

        output->pwm[FS_Wheel_R] = truncate((ptot + pdif) / 2.0);
        output->pwm[FS_Wheel_L] = truncate((ptot - pdif) / 2.0);
    }

    _copterState.pwm[FS_Wheel_L] = output->pwm[FS_Wheel_L];
    _copterState.pwm[FS_Wheel_R] = output->pwm[FS_Wheel_R];

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_GoTo(ControlOutputs_t *output)
{
    uint64_t currUs = Controller_GetUS();

    float phi0 = _copterState.measBuff[_copterState.epochIdx].r[2];

    float dx = _copterState.posController.cmd.x - _copterState.measBuff[_copterState.epochIdx].p[0];
    float dy = _copterState.posController.cmd.y - _copterState.measBuff[_copterState.epochIdx].p[1];
    float dphi = _copterState.posController.cmd.phiSet
        ? (_copterState.posController.cmd.phi - phi0) : 0.0;

    while (dphi > M_PI)
        dphi -= 2.0 * M_PI;
    while (dphi < -M_PI)
        dphi += 2.0 * M_PI;

    if (((fabs(dx) + fabs(dy)) < g_params.path_eps_x) && (fabs(dphi) < g_params.path_eps_phi))
    {
        _copterState.iSum.v = 0.0;
        _copterState.iSum.w = 0.0;

        output->pwm[FS_Wheel_R] = 0.0;
        output->pwm[FS_Wheel_L] = 0.0;
    }
    else
    {
        if (_copterState.posController.newInput
            || ((currUs - _copterState.posController.replanUs) > g_params.path_replan_dt_us))
        {
            float c0 = cos(phi0);
            float s0 = sin(phi0);
            float dxb = c0 * dx + s0 * dy;
            float dyb = -s0 * dx + c0 * dy;

            if (fabs(dyb) < g_params.path_eps_x / 2)
            {
                if (fabs(dxb) < g_params.path_eps_x / 2) // pure rotation
                {
                    float dT = fabs(dphi) / g_params.path_w_max;
                    _copterState.velController.cmd.v = 0;
                    _copterState.velController.cmd.w
                        = fabs(dphi) > g_params.path_eps_phi ? dphi / dT : 0.0;
                }
                else // straight line case
                {
                    float arcLen = fabs(dxb);
                    float dT = arcLen / g_params.path_v_max;

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
                float dT = arcLen / g_params.path_v_max;

                _copterState.velController.cmd.w = theta / dT;
                _copterState.velController.cmd.v = k * _copterState.velController.cmd.w;

                if (_copterState.velController.cmd.v > g_params.path_v_max)
                    _copterState.velController.cmd.v = g_params.path_v_max;
                else if (_copterState.velController.cmd.v < -g_params.path_v_max)
                    _copterState.velController.cmd.v = -g_params.path_v_max;
            }

            _copterState.posController.replanUs = Controller_GetUS();
            _copterState.posController.newInput = 0;
        }

        return FlightScenarioFunc_VelSet(output);
    }

    return FlightScenario_Result_OK;
}

static FlightScenario_Result_t FlightScenarioFunc_TrgTrack(ControlOutputs_t *output)
{
    uint64_t currUs = Controller_GetUS();

    if ((currUs - _copterState.trgTrackController.lastTrgPosUs) > g_params.tag_lost_dt_us)
    {
        _copterState.iSum.v = 0.0f;
        _copterState.iSum.w = 0.0f;

        _copterState.velController.cmd.v = 0.0f;
        _copterState.velController.cmd.w = 0.0f;

        output->pwm[FS_Wheel_R] = 0.0f;
        output->pwm[FS_Wheel_L] = 0.0f;

        return FlightScenario_Result_OK;
    }

    float ex = _copterState.trgTrackController.dx - _copterState.trgTrackController.tdx;
    float ey = _copterState.trgTrackController.dy - _copterState.trgTrackController.tdy;

    if (fabsf(ex) < g_params.tag_eps_x)
        ex = 0.0f;
    if (fabsf(ey) < g_params.tag_eps_y)
        ey = 0.0f;

    // -------- P control: position error -> v, w --------
    // X forward: ex>0  -> move forward
    // Y left:    ey>0  -> rotate CCW (positive w) so target moves toward center
    float v_cmd = g_params.tag_kv * ex;
    float w_cmd = g_params.tag_kw * ey;

    if (v_cmd > g_params.path_v_max)
        v_cmd = g_params.path_v_max;
    if (v_cmd < -g_params.path_v_max)
        v_cmd = -g_params.path_v_max;

    if (w_cmd > g_params.path_w_max)
        w_cmd = g_params.path_w_max;
    if (w_cmd < -g_params.path_w_max)
        w_cmd = -g_params.path_w_max;

    _copterState.velController.cmd.v = v_cmd;
    _copterState.velController.cmd.w = w_cmd;

    return FlightScenarioFunc_VelSet(output);
}
