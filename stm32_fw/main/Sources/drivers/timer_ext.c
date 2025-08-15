#include "timer_ext.h"

// Timer handles
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim4;

static void MX_TIMx_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance);
static TIM_HandleTypeDef *Timer_GatHandle(int dev);

/*******************************************************************************/

void Timer_Init(int dev)
{
    if (dev == 2)
    {
        MX_TIMx_Init(&htim2, TIM2);
    }
    else if (dev == 4)
    {
        MX_TIMx_Init(&htim4, TIM4);
    }
}

uint32_t Timer_GetValue(int dev)
{
    TIM_HandleTypeDef *htim = Timer_GatHandle(dev);
    if (htim)
        return __HAL_TIM_GET_COUNTER(htim);
    return 0;
}

/*******************************************************************************/

static TIM_HandleTypeDef *Timer_GatHandle(int dev)
{
    switch (dev)
    {
    case 2:
        return &htim2;
    case 4:
        return &htim4;
    default:
        return NULL;
    }
}

static void MX_TIMx_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 65535;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Start(htim);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF14_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_TIM4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}