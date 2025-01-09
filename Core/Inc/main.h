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
#include "stm32l4xx_hal.h"

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
#define DIST_SENSOR_AIN1_Pin GPIO_PIN_1
#define DIST_SENSOR_AIN1_GPIO_Port GPIOA
#define DIST_SENSOR_AIN2_Pin GPIO_PIN_2
#define DIST_SENSOR_AIN2_GPIO_Port GPIOA
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOC
#define MOTOR_DIR_RIGHT_Pin GPIO_PIN_8
#define MOTOR_DIR_RIGHT_GPIO_Port GPIOC
#define MOTOR_DIR_LEFT_Pin GPIO_PIN_9
#define MOTOR_DIR_LEFT_GPIO_Port GPIOC
#define MOTOR_PWM_LEFT_Pin GPIO_PIN_10
#define MOTOR_PWM_LEFT_GPIO_Port GPIOA
#define MOTOR_PWM_RIGHT_Pin GPIO_PIN_11
#define MOTOR_PWM_RIGHT_GPIO_Port GPIOA
#define DIGITAL_IN6_Pin GPIO_PIN_15
#define DIGITAL_IN6_GPIO_Port GPIOA
#define DIGITAL_IN5_Pin GPIO_PIN_10
#define DIGITAL_IN5_GPIO_Port GPIOC
#define DIGITAL_IN4_Pin GPIO_PIN_11
#define DIGITAL_IN4_GPIO_Port GPIOC
#define DIGITAL_IN3_Pin GPIO_PIN_12
#define DIGITAL_IN3_GPIO_Port GPIOC
#define DIGITAL_IN2_Pin GPIO_PIN_2
#define DIGITAL_IN2_GPIO_Port GPIOD
#define DIGITAL_IN1_Pin GPIO_PIN_4
#define DIGITAL_IN1_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
