#ifndef _MHELEPRS_H_
#define _MHELEPRS_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FS_NUM_AXIS 3

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
    float x;
    float y;
    float z;
    float w;
} Quaternion_t;

typedef struct
{
    int n;
    int windowSz;
    float val;
} RMA_t;

void FS_QuatToRot(Matrix3D_t *r, Quaternion_t *q);
void FS_ConjQuat(Quaternion_t *q_, Quaternion_t *q);
void FS_QuatMul(Quaternion_t *r, Quaternion_t *q1, Quaternion_t *q2);
void FS_NormQuat(Quaternion_t *q);
void FS_VecRotQuat(Vec3D_t *x, Quaternion_t *q);
void FS_MatMulVec(Vec3D_t *y, Matrix3D_t *yWx, Vec3D_t *x);
void FS_MatTranspose(Matrix3D_t *r);
void FS_MatInitEye(Matrix3D_t *r);
void FS_ScaleVec(float a, Vec3D_t *x);
void FS_ZeroVec(Vec3D_t *x);
float FS_NormVec(Vec3D_t *a);

void FS_Integate(float *y, float y_, float x, float x_, float h);
void FS_Integate3D(Vec3D_t *y, Vec3D_t *y_, Vec3D_t *x, Vec3D_t *x_, float h);

void FS_RmaUpdate(RMA_t *a, float val);

void FS_Sigmoid(float x, float *y, float *yDot, float *yDotDot);

int FS_SolveLS(uint32_t N, float *A, float *b, float *x);

#ifdef __cplusplus
}
#endif

#endif //_MHELEPRS_H_