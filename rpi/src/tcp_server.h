#pragma once

#include <cstddef>
#include <cstdint>

int P2pLink_Init(uint16_t port);

#ifdef __cplusplus
extern "C"
{
#endif

    size_t P2pLink_Read(int dev, uint8_t *buff, size_t count);
    size_t P2pLink_Write(int dev, const uint8_t *buff, size_t count);

#ifdef __cplusplus
}
#endif