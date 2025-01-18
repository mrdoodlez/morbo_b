#include "gpio.h"

int GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pins : PB4*/
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    return 0;
}

int GPIO_Set(GPIO_Channel_t ch, int set)
{
    switch (ch)
    {
    case GPIO_Channel_0:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, set ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    default:
        break;
    }

    return -1;
}

int GPIO_Toggle(GPIO_Channel_t ch)
{
    switch (ch)
    {
    case GPIO_Channel_0:
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        break;
    default:
        break;
    }

    return -1;
}
