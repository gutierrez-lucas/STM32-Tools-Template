/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define display_reset_Pin GPIO_PIN_3
#define display_reset_Port GPIOB
#define main_led_Pin GPIO_PIN_13
#define main_led_GPIO_Port GPIOB
#define sec_led_Pin GPIO_PIN_14
#define sec_led_GPIO_Port GPIOB
#define trace_3_Pin GPIO_PIN_15
#define trace_3_GPIO_Port GPIOB
#define trace_1_Pin GPIO_PIN_9
#define trace_1_GPIO_Port GPIOA
#define trace_2_Pin GPIO_PIN_10
#define trace_2_GPIO_Port GPIOA

#define button_r_Pin GPIO_PIN_5
#define button_r_GPIO_Port GPIOB
#define button_l_Pin GPIO_PIN_15
#define button_l_GPIO_Port GPIOA
#define button_u_Pin GPIO_PIN_11
#define button_u_GPIO_Port GPIOA
#define button_d_Pin GPIO_PIN_8
#define button_d_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
