/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ds1307.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DTMF_BUFFER_SIZE 16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char dtmfBuffer[DTMF_BUFFER_SIZE];  // 用作滑动窗口
uint8_t dtmfIndex = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char DTMF_Decode(uint8_t code) {
    // 按照 0~11 的顺序映射到字符
    const char dtmf_map[12] = {
        '1', '2', '3',
        '4', '5', '6',
        '7', '8', '9',
        '0', '*', '#'
    };

    if (code < 13)
        return dtmf_map[code - 1];
    else
        return '?';  // 未知编码
}

extern UART_HandleTypeDef huart1;
extern char dtmfTimeString[16];
extern DS1307_TimeTypeDef systemTime;

void uart_print(const char* msg) {
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void shift_buffer_and_append(char newChar) {
  // 将前15个字符往前移动一位
  for (int i = 0; i < DTMF_BUFFER_SIZE - 1; ++i) {
    dtmfBuffer[i] = dtmfBuffer[i + 1];
  }
  dtmfBuffer[DTMF_BUFFER_SIZE - 1] = newChar; // 加入新字符
}


void process_dtmf_frame() {
  if (dtmfBuffer[0] == '*' && dtmfBuffer[15] == '#') {
    // 解析时间字符串
    systemTime.year    = (dtmfBuffer[1]  - '0') * 1000 +
                         (dtmfBuffer[2]  - '0') * 100 +
                         (dtmfBuffer[3]  - '0') * 10 +
                         (dtmfBuffer[4]  - '0') - 2000;

    systemTime.month   = (dtmfBuffer[5]  - '0') * 10 + (dtmfBuffer[6]  - '0');
    systemTime.day     = (dtmfBuffer[7]  - '0') * 10 + (dtmfBuffer[8]  - '0');
    systemTime.hours   = (dtmfBuffer[9]  - '0') * 10 + (dtmfBuffer[10] - '0');
    systemTime.minutes = (dtmfBuffer[11] - '0') * 10 + (dtmfBuffer[12] - '0');
    systemTime.seconds = (dtmfBuffer[13] - '0') * 10 + (dtmfBuffer[14] - '0');

    char msg[80];
    snprintf(msg, sizeof(msg), "\r\n[Valid] Time: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
             systemTime.year, systemTime.month, systemTime.day,
             systemTime.hours, systemTime.minutes, systemTime.seconds);
    uart_print(msg);

    DS1307_SetTime(&systemTime);
    uart_print("[RTC] Time updated!\r\n");
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  uint8_t pinStates[4];
  pinStates[0] = HAL_GPIO_ReadPin(DTMF_1_GPIO_Port, DTMF_1_Pin);
  pinStates[1] = HAL_GPIO_ReadPin(DTMF_2_GPIO_Port, DTMF_2_Pin);
  pinStates[2] = HAL_GPIO_ReadPin(DTMF_3_GPIO_Port, DTMF_3_Pin);
  pinStates[3] = HAL_GPIO_ReadPin(DTMF_4_GPIO_Port, DTMF_4_Pin);

  uint8_t value = (pinStates[3] << 3) | (pinStates[2] << 2) | (pinStates[1] << 1) | pinStates[0];
  char decodedChar = DTMF_Decode(value);

  // 滑动窗口追加字符
  shift_buffer_and_append(decodedChar);

  // 打印调试接收字符
  char info[32];
  snprintf(info, sizeof(info), "[DTMF] Received: %c\r\n", decodedChar);
  uart_print(info);

  // 检查并处理完整帧
  process_dtmf_frame();


  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DTMF_TRIG_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
