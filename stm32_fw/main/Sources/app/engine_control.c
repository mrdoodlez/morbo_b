#include "engine_control.h"
#include "pca9685.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "i2c.h"

#define PCA9685_ADDR 0x80

static int _i2cDev;

/******************************************************************************/

int EC_Init(int i2cDev)
{
    _i2cDev = i2cDev;

    GPIO_Set(GPIO_Channel_0, 1);

    PCA9685_Init(_i2cDev, PCA9685_ADDR, 0);
    PCA9685_SetPWMFreq(450);

    for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
    {
        EC_SetThrottle(en, 0.0);
    }

    vTaskDelay(100);

    return 0;
}

void EC_SetThrottle(EC_Engine_t engine, float throttle)
{
    // TODO: remove it
    if (throttle > 0.15)
        throttle = 0.15;

    if (throttle > 1.0)
        throttle = 1.0;

    I2C_Lock(_i2cDev);
    PCA9685_WriteMicroseconds(engine, (1.0 + throttle) * 1.0e3);
    I2C_Unlock(_i2cDev);
}

void EC_Enable(uint8_t en)
{
    GPIO_Set(GPIO_Channel_0, en ? 0 : 1);
}
