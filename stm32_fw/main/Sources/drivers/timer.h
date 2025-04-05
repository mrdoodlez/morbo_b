#ifndef _TIMER_H_
#define _TIMER_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        Timer_OutputCh_0,
        Timer_OutputCh_1,
        Timer_OutputCh_2,
        Timer_OutputCh_3,

        Timer_OutputCh_Tot,

    } Timer_OutputCh_t;

    void Timer_Init(int dev);

    void Timer_SetPWM(int dev, Timer_OutputCh_t oc, float ratio, int reload);

    void Timer_Enable(int dev, Timer_OutputCh_t ch, int en);

    float Timer_GetFreq(int dev);

    uint64_t Timer_GetRuntime(int dev);

#ifdef __cplusplus
}
#endif

#endif //_TIMER_H_
