#ifndef _ENGINE_CONTROL_H_
#define _ENGINE_CONTROL_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    EC_Engine_1 = 0,
    EC_Engine_2,
    EC_Engine_3,
    EC_Engine_4,
} EC_Engine_t;

int EC_Init(int i2cDev);

void EC_SetThrottle(EC_Engine_t engine, float throttle);
void EC_Enable(uint8_t en);

#ifdef __cplusplus
}
#endif

#endif //_ENGINE_CONTROL_H_
