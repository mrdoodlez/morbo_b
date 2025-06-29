#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define I2C_ADDRESS 0x3E
#define I2C_LBUF_LEN 32

extern int _dbg;

typedef enum
{
    I2C_Transaction_None,
    I2C_Transaction_Write,
    I2C_Transaction_Read_AddrSet,
    I2C_Transaction_Read,
} I2C_Transaction_t;

static struct
{
    I2C_HandleTypeDef hi2c1;
    TaskHandle_t txTaskToNotify;
    TaskHandle_t rxTaskToNotify;
    I2C_Transaction_t t;
    uint32_t lastError;

    uint8_t lbuf[I2C_LBUF_LEN];

    SemaphoreHandle_t lock;
    StaticSemaphore_t xMutexBuffer;

} _i2cContext;

static const UBaseType_t txArrayIndex = 0;
static const UBaseType_t rxArrayIndex = 0;

static void MX_I2C1_Init(void);

/******************************************************************************/

void I2C_Init(int dev)
{
    if (dev == 1)
    {
        MX_I2C1_Init();
    }

    _i2cContext.lock = xSemaphoreCreateMutexStatic(&_i2cContext.xMutexBuffer);
    _i2cContext.t = I2C_Transaction_None;
    _i2cContext.lastError = HAL_I2C_ERROR_NONE;
}

void I2C_Lock(int dev)
{
    (void)dev;
    xSemaphoreTake(_i2cContext.lock, 0xFFFFFFFF);
}

void I2C_Unlock(int dev)
{
    (void)dev;
    xSemaphoreGive(_i2cContext.lock);
}

size_t I2C_Read(int dev, uint8_t addr, uint32_t regAddr, I2C_RegAddrLen_t alen,
                uint8_t *buff, size_t count)
{
    if (dev == 1)
    {
        if (HAL_I2C_GetState(&_i2cContext.hi2c1) == HAL_I2C_STATE_READY)
        {
            size_t addrLen = 0;
            switch (alen)
            {
            case I2C_RegAddrLen_8:
                addrLen = 1;
                break;
            case I2C_RegAddrLen_16:
                addrLen = 2;
                break;
            case I2C_RegAddrLen_32:
                addrLen = 4;
                break;
            case I2C_RegAddrLen_0:
            default:
                break;
            }

            if (addrLen != 0)
            {
                _i2cContext.txTaskToNotify = xTaskGetCurrentTaskHandle();

                if (HAL_I2C_Master_Transmit_IT(&_i2cContext.hi2c1, addr,
                                               (uint8_t *)&regAddr, addrLen) != HAL_OK)
                {
                    Error_Handler();
                }

                _i2cContext.t = I2C_Transaction_Read_AddrSet;

                const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
                uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(txArrayIndex, pdTRUE, xMaxBlockTime);

                if (ulNotificationValue != 1)
                {
                    Error_Handler();
                }
            }

            _i2cContext.rxTaskToNotify = xTaskGetCurrentTaskHandle();

            if (HAL_I2C_Master_Receive_IT(&_i2cContext.hi2c1, addr,
                                          buff, count) != HAL_OK)
            {
                Error_Handler();
            }

            _i2cContext.t = I2C_Transaction_Read;

            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
            uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(rxArrayIndex, pdTRUE, xMaxBlockTime);

            _i2cContext.t = I2C_Transaction_None;

            if (ulNotificationValue == 1)
                return count;
        }
        else
        {
            Error_Handler();
        }
    }

    return 0;
}

size_t I2C_Write(int dev, uint8_t addr, uint32_t regAddr, I2C_RegAddrLen_t alen,
                 uint8_t *buff, size_t count)
{
    if (dev == 1)
    {
        if (HAL_I2C_GetState(&_i2cContext.hi2c1) == HAL_I2C_STATE_READY)
        {
            size_t addrLen = 0;
            switch (alen)
            {
            case I2C_RegAddrLen_8:
                addrLen = 1;
                break;
            case I2C_RegAddrLen_16:
                addrLen = 2;
                break;
            case I2C_RegAddrLen_32:
                addrLen = 4;
                break;
            case I2C_RegAddrLen_0:
            default:
                break;
            }

            _i2cContext.txTaskToNotify = xTaskGetCurrentTaskHandle();

            if (addrLen == 0)
            {
                if (HAL_I2C_Master_Transmit_IT(&_i2cContext.hi2c1, addr,
                                               buff, count) != HAL_OK)
                {
                    Error_Handler();
                }
            }
            else
            {
                for (uint32_t i = 0; i < addrLen; i++)
                    _i2cContext.lbuf[i] = ((uint8_t *)&regAddr)[i];

                for (uint32_t i = 0; i < count; i++)
                    _i2cContext.lbuf[addrLen + i] = buff[i];

                if (HAL_I2C_Master_Transmit_IT(&_i2cContext.hi2c1, addr,
                                               _i2cContext.lbuf, addrLen + count) != HAL_OK)
                {
                    Error_Handler();
                }
            }

            _i2cContext.t = I2C_Transaction_Write;

            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); // TODO: use flags or else
            uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed(txArrayIndex, pdTRUE, xMaxBlockTime);

            _i2cContext.t = I2C_Transaction_None;

            if (ulNotificationValue == 1)
                return count;
        }
    }

    return 0;
}

/******************************************************************************/

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
    _i2cContext.hi2c1.Instance = I2C1;
    _i2cContext.hi2c1.Init.Timing = 0x10802D9B; //0x30A0A7FB; //; // 0x30A0A7FB;
    _i2cContext.hi2c1.Init.OwnAddress1 = I2C_ADDRESS;
    _i2cContext.hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _i2cContext.hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _i2cContext.hi2c1.Init.OwnAddress2 = 0;
    _i2cContext.hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    _i2cContext.hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _i2cContext.hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&_i2cContext.hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&_i2cContext.hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&_i2cContext.hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************/

/**
 * @brief This function handles I2C1 event interrupt / I2C1 wake-up interrupt through EXTI line 23.
 */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&_i2cContext.hi2c1);
}

/**
 * @brief This function handles I2C1 error interrupt.
 */
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&_i2cContext.hi2c1);
}

/******************************************************************************/

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C1)
    {
        RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

        /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
        RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
        RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
        HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

        /**I2C1 GPIO Configuration
        PA15     ------> I2C1_SCL
        PB7      ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* I2C1 interrupt Init */
        HAL_NVIC_SetPriority(I2C1_EV_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

        HAL_NVIC_SetPriority(I2C1_ER_IRQn,
                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_I2C1_CLK_DISABLE();

        /**I2C1 GPIO Configuration
        PB8-BOOT0     ------> I2C1_SCL
        PB9     ------> I2C1_SDA
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

        /* I2C1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (_i2cContext.txTaskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(_i2cContext.txTaskToNotify,
                                      txArrayIndex,
                                      &xHigherPriorityTaskWoken);
        _i2cContext.txTaskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (_i2cContext.rxTaskToNotify != 0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(_i2cContext.rxTaskToNotify,
                                      rxArrayIndex,
                                      &xHigherPriorityTaskWoken);
        _i2cContext.rxTaskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
    _i2cContext.lastError = HAL_I2C_GetError(I2cHandle);

    Error_Handler();
}
