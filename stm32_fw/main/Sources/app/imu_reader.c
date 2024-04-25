#include "imu_reader.h"
#include "i2c.h"
#include "motion_di.h"
#include "asm330lhh.h"
#include "FreeRTOS.h"
#include "task.h"

#define ALGO_FREQ			100U /* Algorithm frequency 100Hz */
#define ACC_ODR				((float)ALGO_FREQ)
#define ACC_FS				2 /* FS = <-2g, 2g> */

#define SAMPLETODISCARD		15
#define DECIMATION			1U

static struct
{
	uint8_t i2c;
	char libVersion[35];
	ASM330LHH_Object_t asm330lhh_obj_0;
} _ctx;

static MDI_knobs_t iKnobs;
static MDI_knobs_t *ipKnobs = &iKnobs;

MDI_cal_type_t AccCalMode;
MDI_cal_type_t GyrCalMode;

/******************************************************************************/

static void MotionDI_manager_init(float freq);

static void MotionDI_manager_get_version(char *version, int *length);

static int32_t ASM330LHH_0_Probe();

int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

int32_t GetTick();

void Delay(uint32_t ms);

static int32_t DummyFunc() { return 0; }

/******************************************************************************/

void IMU_Init(uint8_t i2cDev)
{
	_ctx.i2c = i2cDev;

	if (ASM330LHH_0_Probe() != 0)
		; // TODO: handle error

	/* DynamicInclinometer API initialization function */
	MotionDI_manager_init((int)ALGO_FREQ);

	int libVersionLen;

	/* Get library version */
	MotionDI_manager_get_version(_ctx.libVersion, &libVersionLen);

	if (ASM330LHH_ACC_SetOutputDataRate(&_ctx.asm330lhh_obj_0, ACC_ODR) != ASM330LHH_OK)
		; // TODO: handle error

	if (ASM330LHH_ACC_SetFullScale(&_ctx.asm330lhh_obj_0, ACC_FS) != ASM330LHH_OK)
		; // TODO: handle error
}

/******************************************************************************/

int IMU_Read()
{
	uint8_t val;

	if (I2C_Read(_ctx.i2c, ASM330LHH_I2C_ADD_L, 0x0f, I2C_RegAddrLen_8, &val, 1) != 1)
		; // TODO: handle error

	return 0;
}

/**
 * @brief  Initialize the MotionDI engine
 * @param  freq frequency of input data
 * @retval None
 */
void MotionDI_manager_init(float freq)
{
	MotionDI_Initialize(&freq);

	MotionDI_getKnobs(ipKnobs);

	ipKnobs->AccKnob.CalType = MDI_CAL_CONTINUOUS;
	ipKnobs->GyrKnob.CalType = MDI_CAL_CONTINUOUS;

	ipKnobs->AccOrientation[0] = 's';
	ipKnobs->AccOrientation[1] = 'e';
	ipKnobs->AccOrientation[2] = 'u';

	ipKnobs->GyroOrientation[0] = 's';
	ipKnobs->GyroOrientation[1] = 'e';
	ipKnobs->GyroOrientation[2] = 'u';

	ipKnobs->SFKnob.output_type = MDI_ENGINE_OUTPUT_ENU;
	ipKnobs->SFKnob.modx = DECIMATION;

	MotionDI_setKnobs(ipKnobs);

	AccCalMode = ipKnobs->AccKnob.CalType;
	GyrCalMode = ipKnobs->GyrKnob.CalType;
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionDI_manager_get_version(char *version, int *length)
{
	*length = (int)MotionDI_GetLibVersion(version);
}

int32_t ASM330LHH_0_Probe()
{
	ASM330LHH_IO_t io_ctx;
	uint8_t id;
	ASM330LHH_Capabilities_t cap;

	io_ctx.BusType = ASM330LHH_I2C_BUS;
	io_ctx.Address = ASM330LHH_I2C_ADD_L;
	io_ctx.Init = DummyFunc;
	io_ctx.DeInit = DummyFunc;
	io_ctx.ReadReg = ReadReg;
	io_ctx.WriteReg = WriteReg;
	io_ctx.GetTick = GetTick;
	io_ctx.Delay = Delay;

	if (ASM330LHH_RegisterBusIO(&_ctx.asm330lhh_obj_0, &io_ctx) != ASM330LHH_OK)
	{
		return -10;
	}

	if (ASM330LHH_ReadID(&_ctx.asm330lhh_obj_0, &id) != ASM330LHH_OK)
	{
		return -20;
	}

	if (id != ASM330LHH_ID)
	{
		return -30;
	}

	ASM330LHH_GetCapabilities(&_ctx.asm330lhh_obj_0, &cap);
	// TODO: chack cap ?

	if (ASM330LHH_Init(&_ctx.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -40;
	}

	if (ASM330LHH_ACC_Enable(&_ctx.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -50;
	}

	if (ASM330LHH_GYRO_Enable(&_ctx.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -60;
	}

	return 0;
}

int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
	if (I2C_Read(_ctx.i2c, devAddr, reg, I2C_RegAddrLen_8, pData, length) != 1)
		; // TODO: handle error

	return 0;
}

int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
	if (I2C_Write(_ctx.i2c, devAddr, reg, I2C_RegAddrLen_8, pData, length) != 1)
		; // TODO: handle error

	return 0;
}

int32_t GetTick()
{
	return HAL_GetTick();
}

void Delay(uint32_t ms)
{
	vTaskDelay(ms);
}
