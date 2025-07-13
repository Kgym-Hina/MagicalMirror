/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define SCREEN_RST_Pin GPIO_PIN_0
#define SCREEN_RST_GPIO_Port GPIOB
#define SCREEN_DC_Pin GPIO_PIN_1
#define SCREEN_DC_GPIO_Port GPIOB
#define SCREEN_LED_Pin GPIO_PIN_11
#define SCREEN_LED_GPIO_Port GPIOB
#define DTMF_4_Pin GPIO_PIN_9
#define DTMF_4_GPIO_Port GPIOA
#define DTMF_3_Pin GPIO_PIN_10
#define DTMF_3_GPIO_Port GPIOA
#define DTMF_2_Pin GPIO_PIN_11
#define DTMF_2_GPIO_Port GPIOA
#define DTMF_1_Pin GPIO_PIN_12
#define DTMF_1_GPIO_Port GPIOA
#define DTMF_TRIG_Pin GPIO_PIN_15
#define DTMF_TRIG_GPIO_Port GPIOA
#define DTMF_TRIG_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
