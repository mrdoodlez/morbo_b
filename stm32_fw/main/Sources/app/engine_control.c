#include "engine_control.h"
#include "spi.h"
#include <string.h>

#define MOSI_PORT				GPIOB
#define MOSI_PIN				GPIO_PIN_5

#define DSHOT_ZERO				0xc0
#define DSHOT_ONE				0xfc

#define DSHOT_MIN_THROTTLE		48
#define DSHOT_MAX_THROTTLE		0x7ff

static int _spiDev;

static void _EC_SelectChannel(uint8_t engine);

static void _DSHOT_SendWord(uint16_t w);

/******************************************************************************/

int EC_Init(int spiDev)
{
	_spiDev = spiDev;

	return 0;
}

void EC_SetThrottle(EC_Engine_t engine, float throttle)
{
	uint16_t dshotWord = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) * throttle
							+ DSHOT_MIN_THROTTLE;

	_EC_SelectChannel(engine);

	_DSHOT_SendWord(dshotWord);
}

/******************************************************************************/

void _EC_SelectChannel(uint8_t engine)
{

}

void _DSHOT_SendWord(uint16_t value)
{
	if (value > DSHOT_MAX_THROTTLE)
		value = DSHOT_MAX_THROTTLE;

	value <<= 1;

	uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
	uint16_t coded = (value << 4) | crc ;

	static const SPI_Word_t padding[] = {0, 0, 0, 0};

	SPI_Word_t bits[20];
	memset(bits, 0, sizeof(bits));

	uint16_t mask = 0x8000;
	for (uint32_t i = 0; i < 16; i++)
	{
		bits[i] = ((coded & mask) == 0) ? DSHOT_ZERO : DSHOT_ONE;
		mask >>= 1;
	}

	SPI_Write(_spiDev, padding, 4);
	SPI_Write(_spiDev, bits, 20);

}