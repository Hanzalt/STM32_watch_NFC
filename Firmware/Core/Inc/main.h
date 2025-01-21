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
#define Button_LT_Pin GPIO_PIN_0
#define Button_LT_GPIO_Port GPIOA
#define Button_LT_EXTI_IRQn EXTI0_1_IRQn
#define Button_LB_Pin GPIO_PIN_1
#define Button_LB_GPIO_Port GPIOA
#define Button_LB_EXTI_IRQn EXTI0_1_IRQn
#define Tecka_J_Pin GPIO_PIN_2
#define Tecka_J_GPIO_Port GPIOA
#define Tecka_D_Pin GPIO_PIN_3
#define Tecka_D_GPIO_Port GPIOA
#define CLK_TM_Pin GPIO_PIN_7
#define CLK_TM_GPIO_Port GPIOA
#define DIO_TM_Pin GPIO_PIN_0
#define DIO_TM_GPIO_Port GPIOB
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOB
#define CHRG_Pin GPIO_PIN_8
#define CHRG_GPIO_Port GPIOA
#define CHRG_EXTI_IRQn EXTI4_15_IRQn
#define DONE_Pin GPIO_PIN_9
#define DONE_GPIO_Port GPIOA
#define Button_R_Pin GPIO_PIN_10
#define Button_R_GPIO_Port GPIOA
#define Button_R_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
