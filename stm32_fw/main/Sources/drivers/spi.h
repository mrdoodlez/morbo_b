#ifndef _SPI_DRIVER_H_
#define _SPI_DRIVER_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t SPI_Word_t;

typedef enum
{
    SPI_Slave_None,
    SPI_Slave_CS0,
    SPI_Slave_CS1,
} SPI_Slave_t;

void SPI_Init(int dev);

int SPI_Transaction(int dev, SPI_Slave_t slave, const SPI_Word_t *txBuff,
    size_t txCount, SPI_Word_t *rxBuff, size_t rxCount);

#ifdef __cplusplus
}
#endif

#endif //_SPI_DRIVER_H_