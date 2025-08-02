/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    UART/UART_TwoBoards_ComIT/Inc/main.h
 * @author  MCD Application Team
 * @brief   Header for main.c module
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

#define IMU_BUS 1
#define ADC_DEV 1
#define TIM_DEV 1
#define EC_BUS TIM_DEV
#define HIP_SERIAL 1

#define TIM_DEV_L 2
#define TIM_DEV_R 4

#define FUSION_FREQ 50
#define CALIB_FREQ 50

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
