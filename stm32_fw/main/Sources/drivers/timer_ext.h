#ifndef _TIMER_H_
#define _TIMER_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void Timer_Init(int dev);

    uint32_t Timer_GetValue(int dev);

#ifdef __cplusplus
}
#endif

#endif //_TIMER_H_