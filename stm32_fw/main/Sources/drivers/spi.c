#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static SPI_HandleTypeDef hspi1;

static TaskHandle_t taskToNotify = NULL;
static const UBaseType_t arrayIndex = 0;

static SPI_Word_t _txBuff[16];
static SPI_Word_t _rxBuff[16];

static void MX_SPI1_Init(void);

/******************************************************************************/

void SPI_Init(int dev)
{
    if (dev == 1)
    {
        MX_SPI1_Init();
    }
}

int SPI_Transaction(int dev, SPI_Slave_t slave, const SPI_Word_t *txBuff,
    size_t txCount, SPI_Word_t *rxBuff, size_t rxCount)
{
    if (dev == 1)
    {
        if (slave == SPI_Slave_CS0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        }
        else if (slave == SPI_Slave_CS1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        }

        if (rxCount != 0)
        {
            memset(_txBuff, 0xff, sizeof(_txBuff));
            memset(_rxBuff, 0xff, sizeof(_rxBuff));

            if (_txBuff != 0)
                memcpy(_txBuff, txBuff, sizeof(SPI_Word_t) * txCount);

            taskToNotify = xTaskGetCurrentTaskHandle();

            if (HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)_txBuff,
                (uint8_t *)_rxBuff, sizeof(SPI_Word_t) * (txCount + rxCount)) != HAL_OK)
            {
                Error_Handler();
            }

            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
            uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(arrayIndex, pdTRUE, xMaxBlockTime);

            if (ulNotificationValue != 1)
            {
                Error_Handler();
            }

            memcpy(rxBuff, &(_rxBuff[txCount]), sizeof(SPI_Word_t) * rxCount);
        }
        else
        {
            taskToNotify = xTaskGetCurrentTaskHandle();

            if (HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)txBuff, sizeof(SPI_Word_t) * txCount) != HAL_OK)
            {
                Error_Handler();
            }

            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
            uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(arrayIndex, pdTRUE, xMaxBlockTime);

            if (ulNotificationValue != 1)
            {
                Error_Handler();
            }
        }

        if (slave == SPI_Slave_CS0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        }
        else if (slave == SPI_Slave_CS1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        }
    }

    return 0;
}

/******************************************************************************/

/**
 * @brief This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}

/******************************************************************************/

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************/

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hspi->Instance == SPI1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* SPI1 interrupt Init */
        HAL_NVIC_SetPriority(SPI1_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);

        /*Configure GPIO NSS pins : PA8 PA9*/
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        /* SPI1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (taskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(taskToNotify,
                                      arrayIndex,
                                      &xHigherPriorityTaskWoken);
        taskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (taskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(taskToNotify,
                                      arrayIndex,
                                      &xHigherPriorityTaskWoken);
        taskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (taskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(taskToNotify,
                                      arrayIndex,
                                      &xHigherPriorityTaskWoken);
        taskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Error_Handler();
}
