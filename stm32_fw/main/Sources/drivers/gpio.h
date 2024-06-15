#ifndef _GPIO_H_
#define _GPIO_H_

#include "main.h"

typedef enum
{
	GPIO_Channel_0,
} GPIO_Channel_t;

int GPIO_Init();

int GPIO_Set(GPIO_Channel_t ch, int set);
int GPIO_Toggle(GPIO_Channel_t ch);

#endif //_GPIO_H_