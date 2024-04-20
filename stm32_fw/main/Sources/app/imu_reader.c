#include "imu_reader.h"
#include "i2c.h"

#define L3DG20_ADDR					0xD6

#define L3DG20_REG_WHO_AM_I			0x0F

#define LSM303DLHC_ADDR				0x32

#define LSM303DLHC_STATUS_REG_A		0x27

static struct
{
	uint8_t i2c;
} _ctx;

/******************************************************************************/

void IMU_Init(uint8_t i2cDev)
{
	_ctx.i2c = i2cDev;
}

int IMU_Read()
{
	uint8_t val;

	if (I2C_Read(_ctx.i2c, L3DG20_ADDR, L3DG20_REG_WHO_AM_I, I2C_RegAddrLen_8, &val, 1) != 1)
		; // TODO: handle error

	if (I2C_Read(_ctx.i2c, LSM303DLHC_ADDR, LSM303DLHC_STATUS_REG_A, I2C_RegAddrLen_8, &val, 1) != 1)
		; // TODO: handle error

	return 0;
}
