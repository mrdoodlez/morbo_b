#ifndef _IMU_READER_H_
#define _IMU_READER_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void IMU_Init(uint8_t i2cDev);

int IMU_Read();

#ifdef __cplusplus
}
#endif

#endif //_IMU_READER_H_