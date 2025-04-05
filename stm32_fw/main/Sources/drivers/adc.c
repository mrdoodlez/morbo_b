#include "adc.h"
#include "FreeRTOS.h"

ADC_HandleTypeDef hadc1;

static struct
{
    uint32_t currChannel;
    char convInProgress;
    struct
    {
        ADC_ConvertCb cb;
    } conversions[ADC_Chan_Tot];

    uint32_t lastErr;
} _adcContext;

static const uint32_t _channels[] =
    {
        ADC_CHANNEL_1,
        ADC_CHANNEL_2};

static void MX_ADC1_Init(void);
static void SelectChannel(ADC_Chan_t ch);

/******************************************************************************/

void ADC_Init(int dev)
{
    if (dev == 1)
    {
        MX_ADC1_Init();

        /* Perform an ADC automatic self-calibration and enable ADC */
        if ((_adcContext.lastErr = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)) != HAL_OK)
        {
            /* Configuration Error */
            Error_Handler();
        }
    }
}

int ADC_Read(int dev, ADC_Chan_t chan, ADC_ConvertCb cb)
{
    if (dev != 1)
        return -1;

    //if (_adcContext.convInProgress)
    //    return -2;

    SelectChannel(chan);

    _adcContext.conversions[chan].cb = cb;

    /* Start the ADC and enable the end of conversion interrupt */
    if ((_adcContext.lastErr = HAL_ADC_Start_IT(&hadc1)) != HAL_OK)
    {
        /* Configuration Error */
        Error_Handler();
    }

    _adcContext.convInProgress = 1;

    return 0;
}

/******************************************************************************/

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    ADC_MultiModeTypeDef multimode = {0};

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    /* CubeMX enables DMA interrupt even if not configured */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}

static void SelectChannel(ADC_Chan_t ch)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Configure Regular Channel
     */
    sConfig.Channel = _channels[ch];
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    _adcContext.currChannel = ch;
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hadc->Instance == ADC1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_ADC12_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* NVIC configuration for ADC interrupt */
        /* Priority: high-priority */
        HAL_NVIC_SetPriority(ADC1_2_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

        if (NVIC_GetEnableIRQ(ADC1_2_IRQn) == 0)
        {
            Error_Handler();
        }

        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_ADC12_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**ADC1 GPIO Configuration
        PA0     ------> ADC1_IN1
        PA1     ------> ADC1_IN2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* Disable the NVIC configuration for ADC interrupt */
        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);

        /* Peripheral clock disable */
        __HAL_RCC_ADC12_CLK_DISABLE();

        /**ADC1 GPIO Configuration
        PA0     ------> ADC1_IN1
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
    }
}

/******************************************************************************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        uint32_t val = HAL_ADC_GetValue(hadc);

        if (_adcContext.conversions[_adcContext.currChannel].cb != 0)
            _adcContext.conversions[_adcContext.currChannel].cb(val);

        _adcContext.convInProgress = 0;
    }
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupt.
 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}
