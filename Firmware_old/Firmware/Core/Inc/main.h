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
#include "stm32l0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_LB_Pin GPIO_PIN_1
#define Button_LB_GPIO_Port GPIOA
#define Button_LB_EXTI_IRQn EXTI0_1_IRQn
#define Row1_Pin GPIO_PIN_2
#define Row1_GPIO_Port GPIOA
#define Row2_Pin GPIO_PIN_3
#define Row2_GPIO_Port GPIOA
#define Row3_Pin GPIO_PIN_4
#define Row3_GPIO_Port GPIOA
#define Row4_Pin GPIO_PIN_5
#define Row4_GPIO_Port GPIOA
#define Row5_Pin GPIO_PIN_6
#define Row5_GPIO_Port GPIOA
#define Row6_Pin GPIO_PIN_7
#define Row6_GPIO_Port GPIOA
#define Accel_Pin GPIO_PIN_0
#define Accel_GPIO_Port GPIOB
#define Accel_EXTI_IRQn EXTI0_1_IRQn
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOB
#define Button_LT_Pin GPIO_PIN_8
#define Button_LT_GPIO_Port GPIOA
#define Button_LT_EXTI_IRQn EXTI4_15_IRQn
#define EN_5V_Pin GPIO_PIN_9
#define EN_5V_GPIO_Port GPIOA
#define Button_R_Pin GPIO_PIN_10
#define Button_R_GPIO_Port GPIOA
#define Button_R_EXTI_IRQn EXTI4_15_IRQn
#define DIN_STM_Pin GPIO_PIN_15
#define DIN_STM_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_3
#define Buzzer_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
