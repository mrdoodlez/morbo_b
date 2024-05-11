#ifndef _SPI_DRIVER_H_
#define _SPI_DRIVER_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t SPI_Word_t;

void SPI_Init(int dev);

size_t SPI_Write(int dev, const SPI_Word_t *buff, size_t count);

size_t SPI_Read(int dev, SPI_Word_t *buff, size_t count);

#ifdef __cplusplus
}
#endif

#endif //_SPI_DRIVER_H_