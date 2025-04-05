#include "timer.h"
#include "FreeRTOS.h"

#define TIM_CLOCK 21250000 // = 170000000 / 4 / 2
#define PRESCALER_VALUE (uint32_t)(((SystemCoreClock) / 21250000) - 1)

#define TIM_FREQ 480
#define PERIOD_VALUE (uint32_t)(TIM_CLOCK / TIM_FREQ - 1)

static const uint32_t _channels[] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4};

static uint32_t _running[Timer_OutputCh_Tot];

static TIM_OC_InitTypeDef sConfigOC[Timer_OutputCh_Tot];

static TIM_HandleTypeDef htim1;

static void MX_TIM1_Init(void);
static void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static uint32_t ovfs = 0;

/*******************************************************************************/

void Timer_Init(int dev)
{
    if (dev == 1)
        MX_TIM1_Init();
}

float Timer_GetFreq(int dev)
{
    return (float)TIM_FREQ;
}

void Timer_SetPWM(int dev, Timer_OutputCh_t oc, float ratio, int reload)
{
    uint32_t pulse = (uint32_t)(ratio * PERIOD_VALUE);

    if (!reload)
    {
        sConfigOC[oc].OCMode = TIM_OCMODE_PWM1;
        sConfigOC[oc].Pulse = pulse;
        sConfigOC[oc].OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC[oc].OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC[oc].OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC[oc].OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC[oc].OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(&htim1, &(sConfigOC[oc]),
                                    _channels[oc]) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        switch(oc)
        {
            case Timer_OutputCh_0:
                htim1.Instance->CCR1 = pulse;
                break;
            case Timer_OutputCh_1:
                htim1.Instance->CCR2 = pulse;
                break;
            case Timer_OutputCh_2:
                htim1.Instance->CCR3 = pulse;
                break;
            case Timer_OutputCh_3:
                htim1.Instance->CCR4 = pulse;
                break;
            default:
                break;
        }
    }
}

void Timer_Enable(int dev, Timer_OutputCh_t oc, int en)
{
    if (en && (_running[oc] != en))
    {
        if (HAL_TIM_PWM_Start(&htim1, _channels[oc]) != HAL_OK)
        {
            /* PWM generation Error */
            Error_Handler();
        }

        _running[oc] = 1;
    }
    else if (!en)
    {
        if (HAL_TIM_PWM_Stop(&htim1, _channels[oc]) != HAL_OK)
        {
            /* PWM generation Error */
            Error_Handler();
        }

        // we don't want to stop the timer anyway
        __HAL_TIM_ENABLE(&htim1);

        _running[oc] = 0;
    }
}

uint64_t Timer_GetRuntime(int dev)
{
    if (dev == 1)
    {
        uint32_t ticks = TIM1->CNT;
        uint64_t total_ticks = ((uint64_t)ovfs * (PERIOD_VALUE + 1)) + ticks;
        return (total_ticks * 1000000) / (SystemCoreClock / (PRESCALER_VALUE + 1));
    }
    return 0;
}

/*******************************************************************************/

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = PRESCALER_VALUE;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PERIOD_VALUE;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    for (Timer_OutputCh_t oc = Timer_OutputCh_0; oc <= Timer_OutputCh_3; oc++)
    {
        sConfigOC[oc].OCMode = TIM_OCMODE_PWM1;
        sConfigOC[oc].Pulse = PERIOD_VALUE;
        sConfigOC[oc].OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC[oc].OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC[oc].OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC[oc].OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC[oc].OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(&htim1, &(sConfigOC[oc]),
                                      _channels[oc]) != HAL_OK)
        {
            Error_Handler();
        }
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);

    __HAL_TIM_ENABLE(&htim1);
}

/*******************************************************************************/

/**
 * @brief TIM_PWM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm->Instance == TIM1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

        __HAL_TIM_ENABLE_IT(htim_pwm, TIM_IT_UPDATE);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM1)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /**TIM1 GPIO Configuration
        PC0     ------> TIM1_CH1
        PC1     ------> TIM1_CH2
        PC2     ------> TIM1_CH3
        PC3     ------> TIM1_CH4
        */

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}
/**
 * @brief TIM_PWM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm->Instance == TIM1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
}

/**
 * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
 */
void TIM1_UP_TIM16_IRQHandler(void)
{
    ovfs++;
    HAL_TIM_IRQHandler(&htim1);
}
