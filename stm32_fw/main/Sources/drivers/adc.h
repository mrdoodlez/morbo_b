#ifndef _ADC_DRIVER_H_
#define _ADC_DRIVER_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ADC_Chan_0,
        ADC_Chan_1,

        ADC_Chan_Tot,
    } ADC_Chan_t;

    typedef void (*ADC_ConvertCb)(uint32_t);

    void ADC_Init(int dev);

    int ADC_Read(int dev, ADC_Chan_t chan, ADC_ConvertCb cb);

#ifdef __cplusplus
}
#endif

#endif //_ADC_DRIVER_H_
