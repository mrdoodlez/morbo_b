#include "scenarios.h"
#include <string.h>
#include "motion_di.h"

#define FS_NUM_EPOCHS 2
typedef enum
{
    FS_StateFlags_MeasValid = 1 << 0,
    FS_StateFlags_StateDotValid = 1 << 1,
    FS_StateFlags_StateValid = 1 << 2,
} FS_StateFlags_t;

typedef struct
{
    float time;
    MDI_output_t imu;

    float a[3];
    float v[3];
    float p[3];

    uint32_t flags;
} FS_State_t;

static struct
{
    FS_State_t measBuff[FS_NUM_EPOCHS];
    uint8_t epochIdx;

    FlightScenario_PAT_t pat;
    uint32_t epochCounter;
} _copterState;

extern int _dbg;

/******************************************************************************/

typedef struct
{
    float x[FS_NUM_AXIS];
} Vec3D_t;

typedef struct
{
    float r[FS_NUM_AXIS][FS_NUM_AXIS];
} Matrix3D_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

static void FS_QuatToRot(Matrix3D_t *r, Quaternion_t *q);
static void FS_MatMulVec(Vec3D_t *y, Matrix3D_t *yWx, Vec3D_t *x);
static void FS_MatTranspose(Matrix3D_t *r);

static void FS_Integate(float *y, float y_, float x, float x_, float h);
static void FS_Integate3D(Vec3D_t *y, Vec3D_t *y_, Vec3D_t *x, Vec3D_t *x_, float h);

void __dbg_hook(int arg)
{
    _dbg = arg;
}

/******************************************************************************/

int FlightScenario_SetInputs(FlightScenario_Input_t type, void *data)
{
    if (type == FlightScenario_Input_Meas)
    {
        struct
        {
            uint64_t time;
            MDI_output_t meas;
        } meas;

        memcpy((void *)&meas, data, sizeof(meas));

        _copterState.measBuff[_copterState.epochIdx].imu = meas.meas;
        _copterState.measBuff[_copterState.epochIdx].time = meas.time * 1.0e-6;

        _copterState.measBuff[_copterState.epochIdx].flags |= FS_StateFlags_MeasValid;
    }
    else if (type == FlightScenario_Input_DebugPwms)
    {
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
            Vec3D_t *ba = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].imu.linear_acceleration;

            Matrix3D_t rot;
            FS_QuatToRot(&rot, (Quaternion_t *)&_copterState.measBuff[_copterState.epochIdx].imu.quaternion);
            FS_MatTranspose(&rot);

            Vec3D_t *a = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].a;
            Vec3D_t *a_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].a;

            FS_MatMulVec(a, &rot, ba);

            float dt = _copterState.measBuff[_copterState.epochIdx].time - _copterState.measBuff[prevIdx].time;

            Vec3D_t *v = (Vec3D_t *)&_copterState.measBuff[_copterState.epochIdx].v;
            Vec3D_t *v_ = (Vec3D_t *)&_copterState.measBuff[prevIdx].v;

            FS_Integate3D(v, v_, a, a_, dt);

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

            _dbg = 5007;
        }
    }

    _copterState.epochIdx = (_copterState.epochIdx + 1) % FS_NUM_EPOCHS;
    _copterState.measBuff[_copterState.epochIdx].flags = 0;

    _copterState.epochCounter++;

    return FlightScenario_Result_None;
}

void FlightScenario_GetPAT(FlightScenario_PAT_t *pat)
{
    memcpy(pat, &_copterState.pat, sizeof(FlightScenario_PAT_t));
}

/******************************************************************************/

static void FS_Integate3D(Vec3D_t *y, Vec3D_t *y_, Vec3D_t *x, Vec3D_t *x_, float h)
{
    _dbg = 5009;

    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        FS_Integate(&y->x[i], y_->x[i], x->x[i], x_->x[i], h);
    }
}

static void FS_Integate(float *y, float y_, float x, float x_, float h)
{
    *y = y_ + (x + x_) * h / 2.0;

    _dbg = 5008;
}

// Quternion to World-to-Body
static void FS_QuatToRot(Matrix3D_t *r, Quaternion_t *q)
{
    r->r[0][0] = 1 - 2 * (q->y * q->y + q->z * q->z);
    r->r[0][1] = 2 * (q->x * q->y + q->w * q->z);
    r->r[0][2] = 2 * (q->x * q->z - q->w * q->y);

    r->r[1][0] = 2 * (q->x * q->y - q->w * q->z);
    r->r[1][1] = 1 - 2 * (q->x * q->x + q->z * q->z);
    r->r[1][2] = 2 * (q->y * q->z + q->w * q->x);

    r->r[2][0] = 2 * (q->x * q->z + q->w * q->y);
    r->r[2][1] = 2 * (q->y * q->z - q->w * q->x);
    r->r[2][2] = 1 - 2 * (q->x * q->x + q->y * q->y);
}

static void FS_MatTranspose(Matrix3D_t *r)
{
    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        for (int j = i + 1; j < FS_NUM_AXIS; j++)
        {
            float temp = r->r[i][j];
            r->r[i][j] = r->r[j][i];
            r->r[j][i] = temp;
        }
    }
}

static void FS_MatMulVec(Vec3D_t *y, Matrix3D_t *yWx, Vec3D_t *x)
{
    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        y->x[i] = 0;
        for (int j = 0; j < FS_NUM_AXIS; j++)
        {
            y->x[i] += yWx->r[i][j] * x->x[j];
        }
    }
}
