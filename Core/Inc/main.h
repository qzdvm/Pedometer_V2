/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_INT1_Pin LL_GPIO_PIN_14
#define ACC_INT1_GPIO_Port GPIOC
#define ACC_INT1_EXTI_IRQn EXTI4_15_IRQn
#define CS_AS3933_Pin LL_GPIO_PIN_2
#define CS_AS3933_GPIO_Port GPIOA
#define CS_ACC_Pin LL_GPIO_PIN_3
#define CS_ACC_GPIO_Port GPIOA
#define CS_LORA_Pin LL_GPIO_PIN_4
#define CS_LORA_GPIO_Port GPIOA
#define DIO0_Pin LL_GPIO_PIN_10
#define DIO0_GPIO_Port GPIOA
#define BTN_Pin LL_GPIO_PIN_15
#define BTN_GPIO_Port GPIOA
#define BTN_EXTI_IRQn EXTI4_15_IRQn
#define CL_DAT_Pin LL_GPIO_PIN_3
#define CL_DAT_GPIO_Port GPIOB
#define DAT_Pin LL_GPIO_PIN_4
#define DAT_GPIO_Port GPIOB
#define WAKE_Pin LL_GPIO_PIN_5
#define WAKE_GPIO_Port GPIOB
#define WAKE_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
