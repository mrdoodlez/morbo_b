#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	I2C_RegAddrLen_0,
	I2C_RegAddrLen_8,
	I2C_RegAddrLen_16,
	I2C_RegAddrLen_32,
} I2C_RegAddrLen_t;

void I2C_Init(int dev);

size_t I2C_Read(int dev, uint8_t addr, uint32_t regAddr, I2C_RegAddrLen_t alen,
	uint8_t *buff, size_t count);

size_t I2C_Write(int dev, uint8_t addr, uint32_t regAddr, I2C_RegAddrLen_t alen,
	uint8_t *buff, size_t count);

#ifdef __cplusplus
}
#endif

#endif //_I2C_DRIVER_H_
