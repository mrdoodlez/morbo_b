#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"

static UART_HandleTypeDef huart1;

static TaskHandle_t txTaskToNotify = NULL;
static const UBaseType_t txArrayIndex = 0;

static TaskHandle_t rxTaskToNotify = NULL;
static const UBaseType_t rxArrayIndex = 0;

static void MX_USART1_UART_Init(void);

int rxErr = HAL_OK;
int txErr = HAL_OK;

/******************************************************************************/

void Serial_Init(int dev)
{
    if (dev == 0)
    {
        MX_USART1_UART_Init();
    }
}

size_t Serial_Read(int dev, uint8_t *buff, size_t count)
{
    if (dev == 0)
    {
        rxTaskToNotify = xTaskGetCurrentTaskHandle();

        rxErr = HAL_OK;
        if ((rxErr = HAL_UART_Receive_IT(&huart1, buff, count)) != HAL_OK)
        {
            Error_Handler();
        }

        const TickType_t xMaxBlockTime = 0xffffffff;
        uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(rxArrayIndex, pdTRUE, xMaxBlockTime);

        if (ulNotificationValue == 1)
            return count;
    }

    return 0;
}

size_t Serial_Write(int dev, uint8_t *buff, size_t count)
{
    if (dev == 0)
    {
        txTaskToNotify = xTaskGetCurrentTaskHandle();

        txErr = HAL_OK;
        if ((txErr = HAL_UART_Transmit_IT(&huart1, buff, count)) != HAL_OK)
        {
            Error_Handler();
        }

        const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
        uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(txArrayIndex, pdTRUE, xMaxBlockTime);

        if (ulNotificationValue == 1)
            return count;
    }

    return 0;
}

/******************************************************************************/

/**
 * @brief This function handles USART1 global interrupt
 *        USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/******************************************************************************/

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600; // 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************/

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (huart->Instance == USART1)
    {
        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /**USART1 GPIO Configuration
        PC4     ------> USART1_TX
        PC5     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Reset peripherals */
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();

        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

        /* USART1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (txTaskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(txTaskToNotify,
                                      txArrayIndex,
                                      &xHigherPriorityTaskWoken);
        txTaskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (rxTaskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(rxTaskToNotify,
                                      rxArrayIndex,
                                      &xHigherPriorityTaskWoken);
        rxTaskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}
