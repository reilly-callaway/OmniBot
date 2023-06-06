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
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOC
#define SW_Pin GPIO_PIN_0
#define SW_GPIO_Port GPIOA
#define MOTOR_C_DIRN_Pin GPIO_PIN_13
#define MOTOR_C_DIRN_GPIO_Port GPIOB
#define MOTOR_B_DIRN_Pin GPIO_PIN_14
#define MOTOR_B_DIRN_GPIO_Port GPIOB
#define MOTOR_A_DIRN_Pin GPIO_PIN_15
#define MOTOR_A_DIRN_GPIO_Port GPIOB
#define MOTOR_C_PWM_Pin GPIO_PIN_8
#define MOTOR_C_PWM_GPIO_Port GPIOA
#define MOTOR_B_PWM_Pin GPIO_PIN_9
#define MOTOR_B_PWM_GPIO_Port GPIOA
#define MOTOR_A_PWM_Pin GPIO_PIN_10
#define MOTOR_A_PWM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
