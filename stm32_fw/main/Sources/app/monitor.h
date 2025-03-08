#ifndef _MONITOR_H_
#define _MONITOR_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void Monitor_Update();

    float Monitor_GetVbat();
    float Monitor_GetCh1();

#ifdef __cplusplus
}
#endif

#endif //_MONITOR_H_