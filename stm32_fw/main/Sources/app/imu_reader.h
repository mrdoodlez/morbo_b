#ifndef _IMU_READER_H_
#define _IMU_READER_H_

#include "main.h"
#include "mhelpers.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        IMU_Mode_Idle,
        IMU_Mode_CalAcc,
        IMU_Mode_CalGyro,
        IMU_Mode_CalMag,
        IMU_Mode_Fusion,
    } IMU_Mode_t;

    typedef enum
    {
        IMU_Sensor_Acc,
        IMU_Sensor_Gyro,
        IMU_Sensor_Mag,
    } IMU_Sensor_t;

    typedef struct
    {
        Matrix3D_t scale;
        Vec3D_t ofsset;
    } IMU_CalData_t;

    void IMU_Init(uint8_t dev);

    int IMU_SetMode(IMU_Mode_t workmode);

    int IMU_SetCalData(IMU_Sensor_t s, IMU_CalData_t *cd);

    int IMU_GetCalData(IMU_Sensor_t s, IMU_CalData_t *cd, uint8_t *status);

    int IMU_GetAxes(IMU_Sensor_t s, Vec3D_t *raw, Vec3D_t *cal);

#ifdef __cplusplus
}
#endif

#endif //_IMU_READER_H_