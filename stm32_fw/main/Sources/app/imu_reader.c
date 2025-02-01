#include "imu_reader.h"
#include "spi.h"
#include "motion_fx.h"
#include "motion_ac.h"
#include "controller.h"
#include "asm330lhh.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define ALGO_FREQ 250
#define ALGO_PERIOD_MS (1000 / ALGO_FREQ)
#define ACC_ODR ((float)ALGO_FREQ)
#define ACC_FS 2 /* FS = <-2g, 2g> */

#define SAMPLETODISCARD 15
#define DECIMATION 1U

#define ALGO_PERIOD_US (1000000U / ALGO_FREQ) /* Algorithm period [us] */
#define FROM_MG_TO_G 0.001f
#define FROM_G_TO_MG 1000.0f
#define FROM_MDPS_TO_DPS 0.001f
#define FROM_DPS_TO_MDPS 1000.0f
#define FROM_MGAUSS_TO_UT50 (0.1f / 50.0f)
#define FROM_UT50_TO_MGAUSS 500.0f

#define GBIAS_ACC_TH_SC (2.0f * 0.000765f)
#define GBIAS_GYRO_TH_SC (2.0f * 0.002f)
#define GBIAS_MAG_TH_SC (2.0f * 0.001500f)

static const uint32_t ReportInterval = 1000U / ALGO_FREQ;
static volatile uint32_t TimeStamp = 0;
static MAC_knobs_t Knobs;

typedef struct imu_reader
{
    int32_t x;
    int32_t y;
    int32_t z;
} MOTION_SENSOR_Axes_t;

struct
{
    uint8_t spi;
    char libVersion[35];
    ASM330LHH_Object_t asm330lhh_obj_0;
    ASM330LHH_Axes_t accAxes;
    ASM330LHH_Axes_t gyrAxes;
    MOTION_SENSOR_Axes_t magAxes;
    MFX_input_t data_in;
    MFX_output_t data_out;
    int64_t lastTimestamp;
    int discardedCount;

    MOTION_SENSOR_Axes_t magOffset;
    uint8_t magCalStatus;
} g_IMU_State;

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

#define MFX_STATE_SIZE (size_t)(2432)
static uint8_t mfxstate[MFX_STATE_SIZE];

static TimerHandle_t imuTimer;
static StaticTimer_t imuTmrBuffer;

/******************************************************************************/

static void MotionFX_manager_init();

static void MotionFX_manager_get_version(char *version, int *length);

static void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);

// static void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out);

static void MotionFX_manager_MagCal_start(int sampletime);

// static void MotionFX_manager_MagCal_stop(int sampletime);

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

void IMU_Init(uint8_t spiDev)
{
    memset(&g_IMU_State, 0, sizeof(g_IMU_State));

    g_IMU_State.spi = spiDev;

    if (ASM330LHH_0_Probe() != 0)
        ; // TODO: handle error

    if (ASM330LHH_ACC_SetOutputDataRate(&g_IMU_State.asm330lhh_obj_0, ACC_ODR) != ASM330LHH_OK)
        ; // TODO: handle error

    if (ASM330LHH_ACC_SetFullScale(&g_IMU_State.asm330lhh_obj_0, ACC_FS) != ASM330LHH_OK)
        ; // TODO: handle error

    /* DynamicInclinometer API initialization function */
    MotionFX_manager_init();

    /* Get library version */
    int libVersionLen;
    MotionFX_manager_get_version(g_IMU_State.libVersion, &libVersionLen);

    /* Enable magnetometer calibration */
    MotionFX_manager_MagCal_start(ALGO_PERIOD_MS);

    /* Test if calibration data are available */
    MFX_MagCal_output_t mag_cal_test;
    MotionFX_MagCal_getParams(&mag_cal_test);

    /* If calibration data are available load HI coefficients */
    if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
    {
        float ans_float;
        ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        g_IMU_State.magOffset.x = (int32_t)ans_float;
        ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        g_IMU_State.magOffset.y = (int32_t)ans_float;
        ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        g_IMU_State.magOffset.z = (int32_t)ans_float;

        g_IMU_State.magCalStatus = 1;
    }

    MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);

    MotionAC_Initialize((uint8_t)enable);

    /* Get current settings and set desired ones */
    MotionAC_GetKnobs(&Knobs);
    Knobs.MoveThresh_g = MOVE_THR_G;
    Knobs.Run6PointCal = (uint8_t)CalibrationMode;
    Knobs.Sample_ms = ReportInterval;
    (void)MotionAC_SetKnobs(&Knobs);

    /* OPTIONAL */
    /* Get library version */
    MotionAC_manager_get_version(LibVersion, &LibVersionLen);

    if ((imuTimer = xTimerCreateStatic("IMU", pdMS_TO_TICKS(ALGO_PERIOD_MS),
                                       pdTRUE, (void *)0, IMU_Process, &imuTmrBuffer)) == NULL)
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
    g_IMU_State.data_in.acc[0] = (float)g_IMU_State.accAxes.x * FROM_MG_TO_G;
    g_IMU_State.data_in.acc[1] = (float)g_IMU_State.accAxes.y * FROM_MG_TO_G;
    g_IMU_State.data_in.acc[2] = (float)g_IMU_State.accAxes.z * FROM_MG_TO_G;

    /* Convert angular velocity from [mdps] to [dps] */
    g_IMU_State.data_in.gyro[0] = (float)g_IMU_State.gyrAxes.x * FROM_MDPS_TO_DPS;
    g_IMU_State.data_in.gyro[1] = (float)g_IMU_State.gyrAxes.y * FROM_MDPS_TO_DPS;
    g_IMU_State.data_in.gyro[2] = (float)g_IMU_State.gyrAxes.z * FROM_MDPS_TO_DPS;

    /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
    g_IMU_State.data_in.mag[0] = (float)g_IMU_State.magAxes.x * FROM_MGAUSS_TO_UT50;
    g_IMU_State.data_in.mag[1] = (float)g_IMU_State.magAxes.y * FROM_MGAUSS_TO_UT50;
    g_IMU_State.data_in.mag[2] = (float)g_IMU_State.magAxes.z * FROM_MGAUSS_TO_UT50;

    uint64_t timestampUs = GetTick() * 1000;
    float delataTime = (timestampUs - g_IMU_State.lastTimestamp) * 1.0e-6;
    g_IMU_State.lastTimestamp = timestampUs;

    if (g_IMU_State.discardedCount == SAMPLETODISCARD)
    {
        MotionFX_manager_run(&g_IMU_State.data_in, &g_IMU_State.data_out, delataTime);

        Controller_NewMeas(&g_IMU_State.data_out);
    }
    else
    {
        g_IMU_State.discardedCount++;
        float_array_set(g_IMU_State.data_out.quaternion, 0, MFX_QNUM_AXES);
        float_array_set(g_IMU_State.data_out.rotation, 0, MFX_NUM_AXES);
        float_array_set(g_IMU_State.data_out.gravity, 0, MFX_NUM_AXES);
        float_array_set(g_IMU_State.data_out.linear_acceleration, 0, MFX_NUM_AXES);
    }
}

/*******************************************************************************/

/**
 * @brief  Initialize the MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_init(void)
{
    if (MFX_STATE_SIZE < MotionFX_GetStateSize())
        Error_Handler();

    MotionFX_initialize((MFXState_t *)mfxstate);

    MotionFX_getKnobs(mfxstate, ipKnobs);

    ipKnobs->acc_orientation[0] = 'e';
    ipKnobs->acc_orientation[1] = 'n';
    ipKnobs->acc_orientation[2] = 'u';

    ipKnobs->gyro_orientation[0] = 'e';
    ipKnobs->gyro_orientation[1] = 'n';
    ipKnobs->gyro_orientation[2] = 'u';

    ipKnobs->mag_orientation[0] = 's';
    ipKnobs->mag_orientation[1] = 'e';
    ipKnobs->mag_orientation[2] = 'u';

    ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
    ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
    ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;

    ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
    ipKnobs->LMode = 1;
    ipKnobs->modx = DECIMATION;

    MotionFX_setKnobs(mfxstate, ipKnobs);
}

/**
 * @brief  Run Motion Sensor Data Fusion algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @param  delta_time Delta time
 * @retval None
 */
void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time)
{
    MotionFX_propagate(mfxstate, data_out, data_in, &delta_time);
    MotionFX_update(mfxstate, data_out, data_in, &delta_time, NULL);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionFX_manager_get_version(char *version, int *length)
{
    *length = (int)MotionFX_GetLibVersion(version);
}

/**
 * @brief  Run magnetometer calibration algorithm
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out)
{
    MotionFX_MagCal_run(data_in);
    MotionFX_MagCal_getParams(data_out);
}

/**
 * @brief  Start magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_start(int sampletime)
{
    MotionFX_MagCal_init(sampletime, 1);
}

/**
 * @brief  Stop magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_stop(int sampletime)
{
    MotionFX_MagCal_init(sampletime, 0);
}

/*******************************************************************************/

int32_t ASM330LHH_0_Probe()
{
    ASM330LHH_IO_t io_ctx;
    uint8_t id;
    ASM330LHH_Capabilities_t cap;

    io_ctx.BusType = ASM330LHH_SPI_4WIRES_BUS;
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
    uint8_t readAddr = (uint8_t)reg | 0x80;
    if (SPI_Transaction(g_IMU_State.spi, SPI_Slave_CS0, (SPI_Word_t *)&readAddr, 1, pData, length) != 0)
        ; // TODO: handle error

    return 0;
}

int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length)
{
    static uint8_t txBuff[16];
    txBuff[0] = reg;
    memcpy(txBuff + 1, pData, length);

    if (SPI_Transaction(g_IMU_State.spi, SPI_Slave_CS0, (SPI_Word_t *)txBuff, length + 1, 0, 0) != 0)
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
