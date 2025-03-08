#include "imu_reader.h"
#include "spi.h"
#include "motion_fx.h"
#include "motion_ac.h"
#include "controller.h"
#include "lsm6dsox.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define SAMPLE_RATIO (FUSION_FREQ / CALIB_FREQ)

#define TIMER_FREQ FUSUIN_FREQ
#define TIMER_PERIOD (1000 / FUSION_FREQ)

/* MOVE_THR_G recommended between 0.15 - 0.30 g, higher value will relax condition on data selection for calibration but
   reduce the accuracy which will be around (moveThresh_g / 10) */
#define MOVE_THR_G 0.15f
#define ACC_FS 4 /* FS = <-4g, 4g> */

#define FROM_MG_TO_G 0.001f
#define FROM_G_TO_MG 1000.0f
#define FROM_MDPS_TO_DPS 0.001f
#define FROM_DPS_TO_MDPS 1000.0f
#define FROM_MGAUSS_TO_UT50 (0.1f / 50.0f)
#define FROM_UT50_TO_MGAUSS 500.0f

#define GBIAS_ACC_TH_SC (2.0f * 0.000765f)
#define GBIAS_GYRO_TH_SC (2.0f * 0.002f)
#define GBIAS_MAG_TH_SC (2.0f * 0.001500f)

#define IMU_NUM_SENSORS (IMU_Sensor_Mag + 1)

static const uint32_t calibInterval = (1000U / CALIB_FREQ);

extern float adafruit_lsm6dsox_acc[];

float *acc_cal = &(adafruit_lsm6dsox_acc[0]);

typedef enum
{
    DYNAMIC_CALIBRATION = 0,
    SIX_POINT_CALIBRATION = 1
} MAC_calibration_mode_t;

typedef enum
{
    MAC_DISABLE_LIB = 0,
    MAC_ENABLE_LIB = 1
} MAC_enable_lib_t;

struct
{
    uint8_t spi;

    char fxLibVersion[35];
    char acLibVersion[35];

    LSM6DSOX_Object_t lsm6dsox_obj_0;

    struct
    {
        Vec3D_t axesRaw;
        Vec3D_t axesCal;
        IMU_CalData_t cal;
        int calStatus;
    } sensorData[IMU_NUM_SENSORS];

    MFX_input_t fxDataIn;
    MFX_output_t fxDataOut;

    MAC_input_t macDataIn;
    MAC_output_t macDataOut;
    uint32_t calibCounter;

    int64_t lastTimestamp;

    IMU_Mode_t workMode;
} g_IMU_State;

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

static MAC_knobs_t Knobs;

#define MFX_STATE_SIZE (size_t)(2432)
static uint8_t mfxstate[MFX_STATE_SIZE];

static TimerHandle_t imuTimer;
static StaticTimer_t imuTmrBuffer;

/******************************************************************************/

extern int _dbg;

void __dbg_hook(int arg)
{
    _dbg = arg;
}

/******************************************************************************/

static void MotionFX_manager_init();

static void MotionFX_manager_get_version(char *version, int *length);

static void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);

static void MotionAC_manager_get_version(char *version, int *length);

/******************************************************************************/

static int32_t LSM6DSOX_0_Probe();

static int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t GetTick();

static void Delay(uint32_t ms);

static int32_t DummyFunc() { return 0; }

/******************************************************************************/

static void IMU_Process(TimerHandle_t xTimer);

static void IMU_GetSamples(IMU_Sensor_t s);

/******************************************************************************/

void IMU_Init(uint8_t spiDev)
{
    memset(&g_IMU_State, 0, sizeof(g_IMU_State));

    g_IMU_State.spi = spiDev;

    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[0][0] = acc_cal[0];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[1][1] = acc_cal[1];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[2][2] = acc_cal[2];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[0] = acc_cal[3];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[1] = acc_cal[4];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[2] = acc_cal[5];

    g_IMU_State.sensorData[IMU_Sensor_Acc].calStatus = 1;

    if (LSM6DSOX_0_Probe() != 0)
        ; // TODO: handle error

    if (LSM6DSOX_ACC_SetOutputDataRate(&g_IMU_State.lsm6dsox_obj_0, (float)FUSION_FREQ) != LSM6DSOX_OK)
        ; // TODO: handle error

    if (LSM6DSOX_ACC_SetFullScale(&g_IMU_State.lsm6dsox_obj_0, ACC_FS) != LSM6DSOX_OK)
        ; // TODO: handle error

    /* DynamicInclinometer API initialization function */
    MotionFX_manager_init();

    /* Get library version */
    int libVersionLen;
    MotionFX_manager_get_version(g_IMU_State.fxLibVersion, &libVersionLen);

    MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);

    /* AccelerometerCalibration API initialization function */
    MotionAC_Initialize(MAC_ENABLE_LIB);

    /* Get current settings and set desired ones */
    MotionAC_GetKnobs(&Knobs);
    Knobs.MoveThresh_g = MOVE_THR_G;
    Knobs.Run6PointCal = DYNAMIC_CALIBRATION;
    Knobs.Sample_ms = calibInterval;
    (void)MotionAC_SetKnobs(&Knobs);

    /* OPTIONAL */
    /* Get library version */
    MotionAC_manager_get_version(g_IMU_State.acLibVersion, &libVersionLen);

    if ((imuTimer = xTimerCreateStatic("IMU", pdMS_TO_TICKS(TIMER_PERIOD),
                                       pdTRUE, (void *)0, IMU_Process, &imuTmrBuffer)) == NULL)
        ; // TODO: handle error

    if (xTimerStart(imuTimer, 0) != pdPASS)
        ; // TODO: handle error
}

int IMU_SetMode(IMU_Mode_t workmode)
{
    g_IMU_State.workMode = workmode;

    return 0;
}

int IMU_GetAxes(IMU_Sensor_t s, Vec3D_t *raw, Vec3D_t *cal)
{
    *raw = g_IMU_State.sensorData[s].axesRaw;
    *cal = g_IMU_State.sensorData[s].axesCal;
    return 0;
}

int IMU_SetCalData(IMU_Sensor_t s, IMU_CalData_t *cd)
{
    return 0;
}

int IMU_GetCalData(IMU_Sensor_t s, IMU_CalData_t *cd, uint8_t *status)
{
    *cd = g_IMU_State.sensorData[s].cal;
    *status = g_IMU_State.sensorData[s].calStatus;
    return 0;
}

/******************************************************************************/

static void IMU_Process(TimerHandle_t xTimer)
{
    for (int s = 0; s < IMU_NUM_SENSORS; s++)
        IMU_GetSamples((IMU_Sensor_t)s);

    uint64_t timestampUs = GetTick() * 1000;
    float delataTime = (timestampUs - g_IMU_State.lastTimestamp) * 1.0e-6;

    if ((g_IMU_State.workMode == IMU_Mode_CalAcc)
        && (g_IMU_State.calibCounter % SAMPLE_RATIO == 0))
    {
        memcpy(&g_IMU_State.macDataIn.Acc,
               g_IMU_State.sensorData[IMU_Sensor_Acc].axesCal.x, sizeof(g_IMU_State.macDataIn.Acc));
        g_IMU_State.macDataIn.TimeStamp = (int)(timestampUs / 1000);

        uint8_t isCalibrated;
        MotionAC_Update(&g_IMU_State.macDataIn, &isCalibrated);
        MotionAC_GetCalParams(&g_IMU_State.macDataOut);

        if (isCalibrated)
        {
            for (int i = 0; i < FS_NUM_AXIS; i++)
            {
                g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[i] = g_IMU_State.macDataOut.AccBias[i];
                g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[i][i] = g_IMU_State.macDataOut.SF_Matrix[i][i];
            }

            g_IMU_State.sensorData[IMU_Sensor_Acc].calStatus = 1;
        }
    }
    else if (g_IMU_State.workMode == IMU_Mode_CalGyro)
    {
    }
    else if (g_IMU_State.workMode == IMU_Mode_Fusion)
    {
        memcpy(&g_IMU_State.fxDataIn.acc,
               g_IMU_State.sensorData[IMU_Sensor_Acc].axesCal.x, sizeof(g_IMU_State.fxDataIn.acc));
        memcpy(&g_IMU_State.fxDataIn.gyro,
               g_IMU_State.sensorData[IMU_Sensor_Gyro].axesCal.x, sizeof(g_IMU_State.fxDataIn.gyro));

        MotionFX_manager_run(&g_IMU_State.fxDataIn, &g_IMU_State.fxDataOut, delataTime);
        Controller_NewMeas(&g_IMU_State.fxDataOut);
    }

    g_IMU_State.lastTimestamp = timestampUs;
    g_IMU_State.calibCounter++;
}

static void IMU_GetSamples(IMU_Sensor_t s)
{
    LSM6DSOX_Axes_t axes;
    if (s == IMU_Sensor_Acc)
    {
        if (LSM6DSOX_ACC_GetAxes(&g_IMU_State.lsm6dsox_obj_0, &axes) != LSM6DSOX_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MG_TO_G;
    }
    else if (s == IMU_Sensor_Gyro)
    {
        if (LSM6DSOX_GYRO_GetAxes(&g_IMU_State.lsm6dsox_obj_0, &axes) != LSM6DSOX_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MDPS_TO_DPS;
    }
    else
        return;

    if (g_IMU_State.sensorData[s].calStatus == 1)
    {
        for (int i = 0; i < FS_NUM_AXIS; i++)
        {
            g_IMU_State.sensorData[s].axesCal.x[i] = g_IMU_State.sensorData[s].axesRaw.x[i]
                - g_IMU_State.sensorData[s].cal.ofsset.x[i];
            g_IMU_State.sensorData[s].axesCal.x[i] *= g_IMU_State.sensorData[s].cal.scale.r[i][i];
        }
    }
    else
    {
        g_IMU_State.sensorData[s].axesCal = g_IMU_State.sensorData[s].axesRaw;
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
    ipKnobs->modx = 1;

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
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval none
 */
void MotionAC_manager_get_version(char *version, int *length)
{
    *length = (int)MotionAC_GetLibVersion(version);
}

/**
 * @brief  Save the calibration parameters in storage
 * @param  data_size  size of data
 * @param  data  pointer of data
 * @retval Will return 0 the if it is success and 1 if it is failure
 */
char MotionAC_SaveCalInNVM(unsigned short int data_size, unsigned int *data)
{
    return (char)1; /* FAILURE: Write to NVM not implemented. */
}

/**
 * @brief  Load the calibration parameters from storage
 * @param  data_size  size of data
 * @param  data  pointer of data
 * @retval Will return 0 the if it is success and 1 if it is failure
 */
char MotionAC_LoadCalFromNVM(unsigned short int data_size, unsigned int *data)
{
    return (char)1; /* FAILURE: Read from NVM not implemented. */
}

/*******************************************************************************/

int32_t LSM6DSOX_0_Probe()
{
    LSM6DSOX_IO_t io_ctx;
    uint8_t id;
    LSM6DSOX_Capabilities_t cap;

    io_ctx.BusType = LSM6DSOX_SPI_4WIRES_BUS;
    io_ctx.Address = LSM6DSOX_I2C_ADD_H;
    io_ctx.Init = DummyFunc;
    io_ctx.DeInit = DummyFunc;
    io_ctx.ReadReg = ReadReg;
    io_ctx.WriteReg = WriteReg;
    io_ctx.GetTick = GetTick;
    io_ctx.Delay = Delay;

    if (LSM6DSOX_RegisterBusIO(&g_IMU_State.lsm6dsox_obj_0, &io_ctx) != LSM6DSOX_OK)
    {
        return -10;
    }

    if (LSM6DSOX_ReadID(&g_IMU_State.lsm6dsox_obj_0, &id) != LSM6DSOX_OK)
    {
        return -20;
    }

    if (id != LSM6DSOX_ID)
    {
        return -30;
    }

    LSM6DSOX_GetCapabilities(&g_IMU_State.lsm6dsox_obj_0, &cap);
    // TODO: chack cap ?

    if (LSM6DSOX_Init(&g_IMU_State.lsm6dsox_obj_0) != LSM6DSOX_OK)
    {
        return -40;
    }

    if (LSM6DSOX_ACC_Enable(&g_IMU_State.lsm6dsox_obj_0) != LSM6DSOX_OK)
    {
        return -50;
    }

    if (LSM6DSOX_GYRO_Enable(&g_IMU_State.lsm6dsox_obj_0) != LSM6DSOX_OK)
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
