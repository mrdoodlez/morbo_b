#include "engine_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"

static uint32_t _timDev;
static float _timPerMs = 0.0;

int EC_Init(int timDev)
{
    _timDev = timDev;

    _timPerMs = 1000.0 / Timer_GetFreq(_timDev);

    for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
    {
        EC_SetThrottle((Timer_OutputCh_t)en, 0.0);
    }

    vTaskDelay(100);

    return 0;
}

void EC_SetThrottle(EC_Engine_t engine, float throttle)
{
    if (throttle > 1.0)
        throttle = 1.0;

    if (throttle > 0.15) // TODO: remoce it!
        throttle = 0.15;

    float duty = (1.0 + throttle) / _timPerMs;

    Timer_SetPWM(_timDev, (Timer_OutputCh_t)engine, duty);
    Timer_Enable(_timDev, (Timer_OutputCh_t)engine, 1);
}

void EC_Enable(uint8_t en)
{
    for (int en = EC_Engine_1; en <= EC_Engine_4; en++)
    {
        EC_SetThrottle((Timer_OutputCh_t)en, en);
    }
}
