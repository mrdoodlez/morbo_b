#include "mhelpers.h"

void FS_Integate3D(Vec3D_t *y, Vec3D_t *y_, Vec3D_t *x, Vec3D_t *x_, float h)
{
    for (int i = 0; i < FS_NUM_AXIS; i++)
    {
        FS_Integate(&y->x[i], y_->x[i], x->x[i], x_->x[i], h);
    }
}

void FS_Integate(float *y, float y_, float x, float x_, float h)
{
    *y = y_ + (x + x_) * h / 2.0;
}

// Quternion to World-to-Body
void FS_QuatToRot(Matrix3D_t *r, Quaternion_t *q)
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

void FS_MatTranspose(Matrix3D_t *r)
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

void FS_MatMulVec(Vec3D_t *y, Matrix3D_t *yWx, Vec3D_t *x)
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