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
#include "stm32g4xx_hal.h"

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
#define IN_1L_Pin GPIO_PIN_0
#define IN_1L_GPIO_Port GPIOC
#define IN_2L_Pin GPIO_PIN_1
#define IN_2L_GPIO_Port GPIOC
#define STNDBY_Pin GPIO_PIN_4
#define STNDBY_GPIO_Port GPIOA
#define IN_1R_Pin GPIO_PIN_6
#define IN_1R_GPIO_Port GPIOA
#define IN_2R_Pin GPIO_PIN_7
#define IN_2R_GPIO_Port GPIOA
#define LEDON_Pin GPIO_PIN_12
#define LEDON_GPIO_Port GPIOB
#define SENSOR12_Pin GPIO_PIN_13
#define SENSOR12_GPIO_Port GPIOB
#define SENSOR11_Pin GPIO_PIN_14
#define SENSOR11_GPIO_Port GPIOB
#define SENSOR10_Pin GPIO_PIN_15
#define SENSOR10_GPIO_Port GPIOB
#define SENSOR9_Pin GPIO_PIN_6
#define SENSOR9_GPIO_Port GPIOC
#define SENSOR8_Pin GPIO_PIN_7
#define SENSOR8_GPIO_Port GPIOC
#define SENSOR7_Pin GPIO_PIN_8
#define SENSOR7_GPIO_Port GPIOC
#define SENSOR6_Pin GPIO_PIN_9
#define SENSOR6_GPIO_Port GPIOC
#define SENSOR5_Pin GPIO_PIN_8
#define SENSOR5_GPIO_Port GPIOA
#define SENSOR4_Pin GPIO_PIN_9
#define SENSOR4_GPIO_Port GPIOA
#define SENSOR3_Pin GPIO_PIN_10
#define SENSOR3_GPIO_Port GPIOA
#define SENSOR2_Pin GPIO_PIN_11
#define SENSOR2_GPIO_Port GPIOA
#define SENSOR1_Pin GPIO_PIN_12
#define SENSOR1_GPIO_Port GPIOA
#define LD_Pin GPIO_PIN_2
#define LD_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
