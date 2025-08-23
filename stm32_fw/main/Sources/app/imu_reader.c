#include "imu_reader.h"
#include "i2c.h"
#include "motion_fx.h"
#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "asm330lhh.h"

#define TIMER_FREQ FUSUIN_FREQ

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

    ASM330LHH_Object_t accgyro;

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

static int ASM330LHH_0_Init();

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

    if (ASM330LHH_0_Init())
        return -10;

    /* DynamicInclinometer API initialization function */
    MotionFX_manager_init();

    /* Get library version */
    int libVersionLen;
    MotionFX_manager_get_version(g_IMU_State.fxLibVersion, &libVersionLen);

    MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);

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

void IMU_Process(MFX_output_t* fxOut)
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
        memcpy(fxOut, &g_IMU_State.fxDataOut, sizeof(MFX_output_t));
    }

    g_IMU_State.lastTimestamp = timestampUs;
    g_IMU_State.calibCounter++;
}

/******************************************************************************/

static void IMU_GetSamples(IMU_Sensor_t s)
{
    if (s == IMU_Sensor_Acc)
    {
        ASM330LHH_Axes_t axes;
        if (ASM330LHH_ACC_GetAxes(&g_IMU_State.accgyro, &axes) != ASM330LHH_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MG_TO_G;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MG_TO_G;
    }
    else if (s == IMU_Sensor_Gyro)
    {
        ASM330LHH_Axes_t axes;
        if (ASM330LHH_GYRO_GetAxes(&g_IMU_State.accgyro, &axes) != ASM330LHH_OK)
            ; // TODO: handle error

        g_IMU_State.sensorData[s].axesRaw.x[0] = (float)axes.x * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[1] = (float)axes.y * FROM_MDPS_TO_DPS;
        g_IMU_State.sensorData[s].axesRaw.x[2] = (float)axes.z * FROM_MDPS_TO_DPS;
    }
    else if (s == IMU_Sensor_Mag)
    {

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

int ASM330LHH_0_Init()
{
    ASM330LHH_IO_t io_ctx;
    uint8_t id;

    /* Configure the accelero driver */
    io_ctx.BusType = ASM330LHH_I2C_BUS; /* I2C */
    io_ctx.Address = ASM330LHH_I2C_ADD_H;
    io_ctx.Init = DummyFunc;
    io_ctx.DeInit = DummyFunc;
    io_ctx.ReadReg = ReadReg;
    io_ctx.WriteReg = WriteReg;
    io_ctx.GetTick = GetTick;
    io_ctx.Delay = Delay;

    if (ASM330LHH_RegisterBusIO(&g_IMU_State.accgyro, &io_ctx) != ASM330LHH_OK)
        return -10;

    if (ASM330LHH_ReadID(&g_IMU_State.accgyro, &id) != ASM330LHH_OK)
        return -20;

    if (id != ASM330LHH_ID)
        return -30;

    if (ASM330LHH_Init(&g_IMU_State.accgyro) != ASM330LHH_OK)
        return -40;

    if (ASM330LHH_ACC_SetOutputDataRate(&g_IMU_State.accgyro, FUSION_FREQ) != ASM330LHH_OK)
        return -50;

    if (ASM330LHH_ACC_SetFullScale(&g_IMU_State.accgyro, ACC_FS) != ASM330LHH_OK)
        return -60;

    if (ASM330LHH_ACC_Enable(&g_IMU_State.accgyro) != ASM330LHH_OK)
        return -70;

    if (ASM330LHH_GYRO_SetOutputDataRate(&g_IMU_State.accgyro, FUSION_FREQ) != ASM330LHH_OK)
        return -80;

    if (ASM330LHH_GYRO_Enable(&g_IMU_State.accgyro) != ASM330LHH_OK)
        return -90;

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
