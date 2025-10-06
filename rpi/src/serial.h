#pragma once

#include <stddef.h>
#include <stdint.h>

#define HIP_SERIAL 0

#ifdef __cplusplus
extern "C"
{
#endif

size_t Serial_Read(int dev, uint8_t* buff, size_t count);

size_t Serial_Write(int dev, uint8_t* buff, size_t count);

#ifdef __cplusplus
}
#endif