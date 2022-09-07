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
#define DigF_Pin GPIO_PIN_13
#define DigF_GPIO_Port GPIOC
#define Dig1_Pin GPIO_PIN_14
#define Dig1_GPIO_Port GPIOC
#define DigC_Pin GPIO_PIN_15
#define DigC_GPIO_Port GPIOC
#define DigB_Pin GPIO_PIN_0
#define DigB_GPIO_Port GPIOC
#define DigD_Pin GPIO_PIN_1
#define DigD_GPIO_Port GPIOC
#define Dig2_Pin GPIO_PIN_2
#define Dig2_GPIO_Port GPIOC
#define DigE_Pin GPIO_PIN_3
#define DigE_GPIO_Port GPIOC
#define DigA_Pin GPIO_PIN_4
#define DigA_GPIO_Port GPIOA
#define DigG_Pin GPIO_PIN_5
#define DigG_GPIO_Port GPIOA
#define Dig3_Pin GPIO_PIN_6
#define Dig3_GPIO_Port GPIOA
#define M3_IN2_Pin GPIO_PIN_12
#define M3_IN2_GPIO_Port GPIOB
#define M3_IN1_Pin GPIO_PIN_13
#define M3_IN1_GPIO_Port GPIOB
#define M4_IN1_Pin GPIO_PIN_14
#define M4_IN1_GPIO_Port GPIOB
#define M4_IN2_Pin GPIO_PIN_15
#define M4_IN2_GPIO_Port GPIOB
#define M1_IN2_Pin GPIO_PIN_6
#define M1_IN2_GPIO_Port GPIOC
#define M1_IN1_Pin GPIO_PIN_7
#define M1_IN1_GPIO_Port GPIOC
#define M2_IN1_Pin GPIO_PIN_8
#define M2_IN1_GPIO_Port GPIOC
#define M2_IN2_Pin GPIO_PIN_9
#define M2_IN2_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
