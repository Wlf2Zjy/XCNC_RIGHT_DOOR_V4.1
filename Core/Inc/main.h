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
#define STEP_DIAG_Pin GPIO_PIN_12
#define STEP_DIAG_GPIO_Port GPIOB
#define STEP_INDEX_Pin GPIO_PIN_13
#define STEP_INDEX_GPIO_Port GPIOB
#define MS1_Pin GPIO_PIN_14
#define MS1_GPIO_Port GPIOB
#define MS2_Pin GPIO_PIN_15
#define MS2_GPIO_Port GPIOB
#define STEP_ENN_Pin GPIO_PIN_8
#define STEP_ENN_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_11
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOA
#define STDBY_Pin GPIO_PIN_15
#define STDBY_GPIO_Port GPIOA
#define SW_RFID_U_Pin GPIO_PIN_3
#define SW_RFID_U_GPIO_Port GPIOB
#define SW_RFID_U_EXTI_IRQn EXTI3_IRQn
#define SW_RFID_D_Pin GPIO_PIN_4
#define SW_RFID_D_GPIO_Port GPIOB
#define SW_RFID_D_EXTI_IRQn EXTI4_IRQn
#define ELECTROMAGNET_RFID_Pin GPIO_PIN_5
#define ELECTROMAGNET_RFID_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
