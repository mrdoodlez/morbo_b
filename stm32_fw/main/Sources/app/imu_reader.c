#include "imu_reader.h"
#include "i2c.h"
#include "motion_di.h"
#include "controller.h"
#include "asm330lhh.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define ALGO_FREQ			100 /* Algorithm frequency 100Hz */
#define ALGO_PERIOD_MS		(1000 / ALGO_FREQ)
#define ACC_ODR				((float)ALGO_FREQ)
#define ACC_FS				2 /* FS = <-2g, 2g> */

#define SAMPLETODISCARD		15
#define DECIMATION			1U

#define ALGO_PERIOD_US		(1000000U / ALGO_FREQ) /* Algorithm period [us] */
#define FROM_MG_TO_G		0.001f
#define FROM_G_TO_MG		1000.0f
#define FROM_MDPS_TO_DPS	0.001f
#define FROM_DPS_TO_MDPS	1000.0f
#define FROM_MGAUSS_TO_UT50	(0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS	500.0f

struct
{
	uint8_t i2c;
	char libVersion[35];
	ASM330LHH_Object_t asm330lhh_obj_0;
	ASM330LHH_Axes_t accAxes;
	ASM330LHH_Axes_t gyrAxes;
	MDI_input_t data_in;
	MDI_output_t data_out;
	int64_t timestamp;
	int discardedCount;
} g_IMU_State;

static MDI_knobs_t iKnobs;
static MDI_knobs_t *ipKnobs = &iKnobs;

static MDI_cal_type_t AccCalMode;
static MDI_cal_type_t GyrCalMode;

static TimerHandle_t imuTimer;
static StaticTimer_t imuTmrBuffer;

/******************************************************************************/

static void MotionDI_manager_init(float freq);

static void MotionDI_manager_get_version(char *version, int *length);

static int32_t ASM330LHH_0_Probe();

static int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t GetTick();

static void Delay(uint32_t ms);

static int32_t DummyFunc() { return 0; }

static void float_array_set(float array[], float value, uint32_t count);

/******************************************************************************/

static void IMU_Process(TimerHandle_t xTimer);

/******************************************************************************/

void IMU_Init(uint8_t i2cDev)
{
	g_IMU_State.i2c = i2cDev;

	if (ASM330LHH_0_Probe() != 0)
		; // TODO: handle error

	/* DynamicInclinometer API initialization function */
	MotionDI_manager_init((int)ALGO_FREQ);

	int libVersionLen;

	/* Get library version */
	MotionDI_manager_get_version(g_IMU_State.libVersion, &libVersionLen);

	if (ASM330LHH_ACC_SetOutputDataRate(&g_IMU_State.asm330lhh_obj_0, ACC_ODR) != ASM330LHH_OK)
		; // TODO: handle error

	if (ASM330LHH_ACC_SetFullScale(&g_IMU_State.asm330lhh_obj_0, ACC_FS) != ASM330LHH_OK)
		; // TODO: handle error

	if ((imuTimer = xTimerCreateStatic("IMU", pdMS_TO_TICKS(ALGO_PERIOD_MS),
				pdTRUE, (void*)0, IMU_Process, &imuTmrBuffer)) == NULL)
		; // TODO: handle error

	if (xTimerStart(imuTimer, 0) != pdPASS)
		; // TODO: handle error	
}

/******************************************************************************/

static void IMU_Process(TimerHandle_t xTimer)
{
	/*
	uint8_t val;
	if (I2C_Read(g_IMU_State.i2c, ASM330LHH_I2C_ADD_L, 0x0f, I2C_RegAddrLen_8, &val, 1) != 1)
			; // TODO: handle error
	*/

	if (ASM330LHH_ACC_GetAxes(&g_IMU_State.asm330lhh_obj_0, &g_IMU_State.accAxes) != ASM330LHH_OK)
		; // TODO: handle error

	if (ASM330LHH_GYRO_GetAxes(&g_IMU_State.asm330lhh_obj_0, &g_IMU_State.gyrAxes) != ASM330LHH_OK)
		; // TODO: handle error

	/* Convert acceleration from [mg] to [g] */
	g_IMU_State.data_in.Acc[0] = (float)g_IMU_State.accAxes.x * FROM_MG_TO_G;
	g_IMU_State.data_in.Acc[1] = (float)g_IMU_State.accAxes.y * FROM_MG_TO_G;
	g_IMU_State.data_in.Acc[2] = (float)g_IMU_State.accAxes.z * FROM_MG_TO_G;

	/* Convert angular velocity from [mdps] to [dps] */
	g_IMU_State.data_in.Gyro[0] = (float)g_IMU_State.gyrAxes.x * FROM_MDPS_TO_DPS;
	g_IMU_State.data_in.Gyro[1] = (float)g_IMU_State.gyrAxes.y * FROM_MDPS_TO_DPS;
	g_IMU_State.data_in.Gyro[2] = (float)g_IMU_State.gyrAxes.z * FROM_MDPS_TO_DPS;

	g_IMU_State.data_in.Timestamp = g_IMU_State.timestamp;
	g_IMU_State.timestamp += ALGO_PERIOD_US;

	if (g_IMU_State.discardedCount == SAMPLETODISCARD)
	{
		MotionDI_update(&g_IMU_State.data_out, &g_IMU_State.data_in);

		Controller_NewMeas(&g_IMU_State.data_out);
	}
	else
	{
		g_IMU_State.discardedCount++;
		float_array_set(g_IMU_State.data_out.quaternion, 0, MDI_QNUM_AXES);
		float_array_set(g_IMU_State.data_out.rotation, 0, MDI_NUM_AXES);
		float_array_set(g_IMU_State.data_out.gravity, 0, MDI_NUM_AXES);
		float_array_set(g_IMU_State.data_out.linear_acceleration, 0, MDI_NUM_AXES);
	}
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
	io_ctx.Address = ASM330LHH_I2C_ADD_H;
	io_ctx.Init = DummyFunc;
	io_ctx.DeInit = DummyFunc;
	io_ctx.ReadReg = ReadReg;
	io_ctx.WriteReg = WriteReg;
	io_ctx.GetTick = GetTick;
	io_ctx.Delay = Delay;

	if (ASM330LHH_RegisterBusIO(&g_IMU_State.asm330lhh_obj_0, &io_ctx) != ASM330LHH_OK)
	{
		return -10;
	}

	if (ASM330LHH_ReadID(&g_IMU_State.asm330lhh_obj_0, &id) != ASM330LHH_OK)
	{
		return -20;
	}

	if (id != ASM330LHH_ID)
	{
		return -30;
	}

	ASM330LHH_GetCapabilities(&g_IMU_State.asm330lhh_obj_0, &cap);
	// TODO: chack cap ?

	if (ASM330LHH_Init(&g_IMU_State.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -40;
	}

	if (ASM330LHH_ACC_Enable(&g_IMU_State.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -50;
	}

	if (ASM330LHH_GYRO_Enable(&g_IMU_State.asm330lhh_obj_0) != ASM330LHH_OK)
	{
		return -60;
	}

	return 0;
}

int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
	if (I2C_Read(g_IMU_State.i2c, devAddr, reg, I2C_RegAddrLen_8, pData, length) != 1)
		; // TODO: handle error

	return 0;
}

int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
	if (I2C_Write(g_IMU_State.i2c, devAddr, reg, I2C_RegAddrLen_8, pData, length) != 1)
		; // TODO: handle error

	return 0;
}

int32_t GetTick()
{
	return xTaskGetTickCount();
}

void Delay(uint32_t ms)
{
	vTaskDelay(ms);
}

/**
 * @brief  Set float array items to value
 * @param  array Destination float array
 * @param  value Set to this value
 * @param  count Number of items to be set
 * @retval None
 */
void float_array_set(float array[], float value, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++)
	{
		array[i] = value;
	}
}
