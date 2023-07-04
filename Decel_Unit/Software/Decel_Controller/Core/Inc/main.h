/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define CS_ADC_HV1_ELECTRODE_CURRENT_Pin GPIO_PIN_3
#define CS_ADC_HV1_ELECTRODE_CURRENT_GPIO_Port GPIOA
#define GAIN_B_Pin GPIO_PIN_4
#define GAIN_B_GPIO_Port GPIOA
#define GAIN_A_Pin GPIO_PIN_5
#define GAIN_A_GPIO_Port GPIOA
#define CS_ADC_HV1_FG_Pin GPIO_PIN_6
#define CS_ADC_HV1_FG_GPIO_Port GPIOA
#define RDY_ADC_SLOW_HV1_FB_Pin GPIO_PIN_7
#define RDY_ADC_SLOW_HV1_FB_GPIO_Port GPIOA
#define TEMP_MON_ADC_FB_Pin GPIO_PIN_4
#define TEMP_MON_ADC_FB_GPIO_Port GPIOC
#define TEMP_MON_VREF_Pin GPIO_PIN_5
#define TEMP_MON_VREF_GPIO_Port GPIOC
#define CS_DISPLAY_Pin GPIO_PIN_0
#define CS_DISPLAY_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOB
#define CS_ADC_SLOW_HV1_FB_Pin GPIO_PIN_12
#define CS_ADC_SLOW_HV1_FB_GPIO_Port GPIOB
#define CS_ADC_FAST_HV1_FB_Pin GPIO_PIN_6
#define CS_ADC_FAST_HV1_FB_GPIO_Port GPIOC
#define CS_TC_RES_STACK_Pin GPIO_PIN_7
#define CS_TC_RES_STACK_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_8
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_9
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_8
#define LED_3_GPIO_Port GPIOA
#define HV_ENABLE_RETURN_Pin GPIO_PIN_15
#define HV_ENABLE_RETURN_GPIO_Port GPIOA
#define HV_ENABLE_SOFTWARE_Pin GPIO_PIN_11
#define HV_ENABLE_SOFTWARE_GPIO_Port GPIOC
#define SYNC_DAC_ULTRAVOLT_CTRL_Pin GPIO_PIN_12
#define SYNC_DAC_ULTRAVOLT_CTRL_GPIO_Port GPIOC
#define CS_ADC_TEMP_DAC_HV1_CTRL_Pin GPIO_PIN_2
#define CS_ADC_TEMP_DAC_HV1_CTRL_GPIO_Port GPIOD
#define SYNC_DAC_HV1_CTRL_Pin GPIO_PIN_6
#define SYNC_DAC_HV1_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
