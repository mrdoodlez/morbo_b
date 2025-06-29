#include "engine_control.h"
#include "pca9685.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "i2c.h"

#define PCA9685_ADDR 0x80

#define PWM_FREQ 1500
#define PERIOD (1.0e6f / PWM_FREQ)

#define PWM_CHANNELS 4

static int _i2cDev;

/******************************************************************************/

int EC_Init(int i2cDev)
{
    _i2cDev = i2cDev;

    GPIO_Set(GPIO_Channel_0, 1);

    PCA9685_Init(_i2cDev, PCA9685_ADDR, 0);
    PCA9685_SetPWMFreq(PWM_FREQ);

    for (int ch = 0; ch < PWM_CHANNELS; ch++)
    {
        PCA9685_WriteMicroseconds(ch, 0);
    }

    return 0;
}

void EC_SetThrottle(EC_Engine_t engine, float throttle)
{
    if (throttle < -1.0)
        throttle = -1.0;

    if (throttle > 1.0)
        throttle = 1.0;

    I2C_Lock(_i2cDev);

    if (throttle > 0)
    {
        PCA9685_WriteMicroseconds((int)engine * 2 + 1, 0);
        PCA9685_WriteMicroseconds((int)engine * 2, throttle * PERIOD);
    }
    else
    {
        throttle = -throttle;
        PCA9685_WriteMicroseconds((int)engine * 2, 0);
        PCA9685_WriteMicroseconds((int)engine * 2 + 1, throttle * PERIOD);
    }

    I2C_Unlock(_i2cDev);
}

void EC_Enable(uint8_t en)
{
    GPIO_Set(GPIO_Channel_0, en ? 0 : 1);
}
