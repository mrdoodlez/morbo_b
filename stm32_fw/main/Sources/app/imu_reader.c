#include "imu_reader.h"
#include "i2c.h"
#include "motion_fx.h"
#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "a3g4250d.h"
#include "ism303dac.h"

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

struct
{
    uint8_t i2c;

    char fxLibVersion[35];
    char acLibVersion[35];

    ISM303DAC_ACC_Object_t acc;
    ISM303DAC_MAG_Object_t mag;
    A3G4250D_Object_t gyro;

    struct
    {
        Vec3D_t axesRaw;
        Vec3D_t axesCal;
        IMU_CalData_t cal;
        int calStatus;
    } sensorData[IMU_NUM_SENSORS];

    MFX_input_t fxDataIn;
    MFX_output_t fxDataOut;

    uint32_t calibCounter;

    int64_t lastTimestamp;

    IMU_Mode_t workMode;
} g_IMU_State;

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

#define MFX_STATE_SIZE (size_t)(2432)
static uint8_t mfxstate[MFX_STATE_SIZE];

/******************************************************************************/

extern int _dbg;

void __dbg_hook(int arg)
{
    _dbg = arg;
}

/******************************************************************************/

static int ISM303DAC_ACC_0_Init();

static int ISM303DAC_MAG_0_Init();

static int A3G4250D_0_Init();

/******************************************************************************/

static void MotionFX_manager_init();

static void MotionFX_manager_get_version(char *version, int *length);

static void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);

/******************************************************************************/

static int32_t ReadReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t WriteReg(uint16_t devAddr, uint16_t reg, uint8_t *pData, uint16_t length);

static int32_t GetTick();

static void Delay(uint32_t ms);

static int32_t DummyFunc() { return 0; }

/******************************************************************************/

static void IMU_GetSamples(IMU_Sensor_t s);

/******************************************************************************/

int IMU_Init(uint8_t i2cDev)
{
    memset(&g_IMU_State, 0, sizeof(g_IMU_State));

    g_IMU_State.i2c = i2cDev;

    /*
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[0][0] = acc_cal[0];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[1][1] = acc_cal[1];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.scale.r[2][2] = acc_cal[2];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[0] = acc_cal[3];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[1] = acc_cal[4];
    g_IMU_State.sensorData[IMU_Sensor_Acc].cal.ofsset.x[2] = acc_cal[5];

    g_IMU_State.sensorData[IMU_Sensor_Acc].calStatus = 1;
    */

    if (ISM303DAC_ACC_0_Init())
        return -10;

    if (ISM303DAC_MAG_0_Init())
        return -20;

    if (A3G4250D_0_Init())
        return -30;

    /* DynamicInclinometer API initialization function */
    MotionFX_manager_init();

    /* Get library version */
    int libVersionLen;
    MotionFX_manager_get_version(g_IMU_State.fxLibVersion, &libVersionLen);

    MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);

    return 0;
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

void IMU_Process()
{
    for (int s = 0; s < IMU_NUM_SENSORS; s++)
        IMU_GetSamples((IMU_Sensor_t)s);

    uint64_t timestampUs = GetTick() * 1000;
    float delataTime = (timestampUs - g_IMU_State.lastTimestamp) * 1.0e-6;

    if (g_IMU_State.workMode == IMU_Mode_CalAcc)
    {
    }
    else if (g_IMU_State.workMode == IMU_Mode_CalGyro)
    {
    }
    else if (g_IMU_State.workMode == IMU_Mode_CalMag)
    {
    }
    else if (g_IMU_State.workMode == IMU_Mode_Fusion)
    {
        memcpy(&g_IMU_State.fxDataIn.acc,
               g_IMU_State.sensorData[IMU_Sensor_Acc].axesCal.x, sizeof(g_IMU_State.fxDataIn.acc));
        memcpy(&g_IMU_State.fxDataIn.gyro,
               g_IMU_State.sensorData[IMU_Sensor_Gyro].axesCal.x, sizeof(g_IMU_State.fxDataIn.gyro));
        memcpy(&g_IMU_State.fxDataIn.mag,
               g_IMU_State.sensorData[IMU_Sensor_Mag].axesCal.x, sizeof(g_IMU_State.fxDataIn.mag));

        MotionFX_manager_run(&g_IMU_State.fxDataIn, &g_IMU_State.fxDataOut, delataTime);
    }

    g_IMU_State.lastTimestamp = timestampUs;
    g_IMU_State.calibCounter++;
}

/******************************************************************************/

static void IMU_GetSamples(IMU_Sensor_t s)
{
    if (s == IMU_Sensor_Acc)
    {
        ISM303DAC_Axes_t axes;
        if (ISM303DAC_ACC_GetAxes(&g_IMU_State.acc, &axes) != ISM303DAC_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MG_TO_G;
    }
    else if (s == IMU_Sensor_Gyro)
    {
        A3G4250D_Axes_t axes;
        if (A3G4250D_GYRO_GetAxes(&g_IMU_State.gyro, &axes) != A3G4250D_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MDPS_TO_DPS;
    }
    else if (s == IMU_Sensor_Mag)
    {
        ISM303DAC_Axes_t axes;
        if (ISM303DAC_MAG_GetAxes(&g_IMU_State.mag, &axes) != ISM303DAC_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MGAUSS_TO_UT50;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MGAUSS_TO_UT50;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MGAUSS_TO_UT50;
    }
    else
        return;

    if (g_IMU_State.sensorData[s].calStatus == 1)
    {
        for (int i = 0; i < FS_NUM_AXIS; i++)
        {
            g_IMU_State.sensorData[s].axesCal.x[i] = g_IMU_State.sensorData[s].axesRaw.x[i] - g_IMU_State.sensorData[s].cal.ofsset.x[i];
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

    ipKnobs->acc_orientation[0] = 'n';
    ipKnobs->acc_orientation[1] = 'e';
    ipKnobs->acc_orientation[2] = 'd';

    ipKnobs->gyro_orientation[0] = 'n';
    ipKnobs->gyro_orientation[1] = 'e';
    ipKnobs->gyro_orientation[2] = 'd';

    ipKnobs->mag_orientation[0] = 'n';
    ipKnobs->mag_orientation[1] = 'e';
    ipKnobs->mag_orientation[2] = 'd';

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

/*******************************************************************************/

int ISM303DAC_ACC_0_Init()
{
    ISM303DAC_IO_t io_ctx;
    uint8_t id;

    /* Configure the accelero driver */
    io_ctx.BusType = ISM303DAC_I2C_BUS; /* I2C */
    io_ctx.Address = ISM303DAC_I2C_ADD_XL;
    io_ctx.Init = DummyFunc;
    io_ctx.DeInit = DummyFunc;
    io_ctx.ReadReg = ReadReg;
    io_ctx.WriteReg = WriteReg;
    io_ctx.GetTick = GetTick;
    io_ctx.Delay = Delay;

    if (ISM303DAC_ACC_RegisterBusIO(&g_IMU_State.acc, &io_ctx) != ISM303DAC_OK)
        return -10;

    if (ISM303DAC_ACC_ReadID(&g_IMU_State.acc, &id) != ISM303DAC_OK)
        return -20;

    if (id != ISM303DAC_ID_XL)
        return -30;

    if (ISM303DAC_ACC_Init(&g_IMU_State.acc) != ISM303DAC_OK)
        return -40;

    if (ISM303DAC_ACC_Enable(&g_IMU_State.acc) != ISM303DAC_OK)
        return -50;

    return 0;
}

int ISM303DAC_MAG_0_Init()
{
    ISM303DAC_IO_t io_ctx;
    uint8_t id;

    /* Configure the accelero driver */
    io_ctx.BusType = ISM303DAC_I2C_BUS; /* I2C */
    io_ctx.Address = ISM303DAC_I2C_ADD_MG;
    io_ctx.Init = DummyFunc;
    io_ctx.DeInit = DummyFunc;
    io_ctx.ReadReg = ReadReg;
    io_ctx.WriteReg = WriteReg;
    io_ctx.GetTick = GetTick;
    io_ctx.Delay = Delay;

    if (ISM303DAC_MAG_RegisterBusIO(&g_IMU_State.mag, &io_ctx) != ISM303DAC_OK)
        return -10;

    if (ISM303DAC_MAG_ReadID(&g_IMU_State.mag, &id) != ISM303DAC_OK)
        return -20;

    if (id != ISM303DAC_ID_MG)
        return -30;

    if (ISM303DAC_MAG_Init(&g_IMU_State.mag) != ISM303DAC_OK)
        return -40;

    if (ISM303DAC_MAG_Enable(&g_IMU_State.mag) != ISM303DAC_OK)
        return -50;

    return 0;
}

int A3G4250D_0_Init()
{
    A3G4250D_IO_t io_ctx;
    uint8_t id;

    /* Configure the accelero driver */
    io_ctx.BusType = A3G4250D_I2C_BUS; /* I2C */
    io_ctx.Address = A3G4250D_I2C_ADD_L;
    io_ctx.Init = DummyFunc;
    io_ctx.DeInit = DummyFunc;
    io_ctx.ReadReg = ReadReg;
    io_ctx.WriteReg = WriteReg;
    io_ctx.GetTick = GetTick;
    io_ctx.Delay = Delay;

    if (A3G4250D_RegisterBusIO(&g_IMU_State.gyro, &io_ctx) != A3G4250D_OK)
        return -10;

    if (A3G4250D_ReadID(&g_IMU_State.gyro, &id) != A3G4250D_OK)
        return -20;

    if (id != A3G4250D_ID)
        return -30;

    if (A3G4250D_Init(&g_IMU_State.gyro) != ISM303DAC_OK)
        return -40;

    if (A3G4250D_GYRO_Enable(&g_IMU_State.gyro) != ISM303DAC_OK)
        return -50;

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
