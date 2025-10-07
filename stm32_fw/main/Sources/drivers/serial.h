#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "../platform/main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Serial_Init(int dev);

size_t Serial_Read(int dev, uint8_t* buff, size_t count);

size_t Serial_Write(int dev, const uint8_t* buff, size_t count);

#ifdef __cplusplus
}
#endif

#endif // _SERIAL_H_
