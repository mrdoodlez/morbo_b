#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

int Serial_Init(int dev, const char* const path);

size_t Serial_Read(int dev, uint8_t* buff, size_t count);

size_t Serial_Write(int dev, const uint8_t* buff, size_t count);

#ifdef __cplusplus
}
#endif