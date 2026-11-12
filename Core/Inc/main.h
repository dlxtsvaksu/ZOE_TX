/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define SF_Pin GPIO_PIN_13
#define SF_GPIO_Port GPIOC
#define SD1_Pin GPIO_PIN_14
#define SD1_GPIO_Port GPIOC
#define SD2_Pin GPIO_PIN_15
#define SD2_GPIO_Port GPIOC
#define SE_Pin GPIO_PIN_7
#define SE_GPIO_Port GPIOA
#define SC1_Pin GPIO_PIN_0
#define SC1_GPIO_Port GPIOB
#define SC2_Pin GPIO_PIN_1
#define SC2_GPIO_Port GPIOB
#define CHARGE_Pin GPIO_PIN_12
#define CHARGE_GPIO_Port GPIOB
#define PWR_DETECT_Pin GPIO_PIN_13
#define PWR_DETECT_GPIO_Port GPIOB
#define PWR_EN_Pin GPIO_PIN_14
#define PWR_EN_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOB
#define SA_Pin GPIO_PIN_8
#define SA_GPIO_Port GPIOA
#define TRIM8_Pin GPIO_PIN_9
#define TRIM8_GPIO_Port GPIOA
#define TRIM7_Pin GPIO_PIN_10
#define TRIM7_GPIO_Port GPIOA
#define USB_DETECT_Pin GPIO_PIN_13
#define USB_DETECT_GPIO_Port GPIOA
#define TRIM_MODE_Pin GPIO_PIN_14
#define TRIM_MODE_GPIO_Port GPIOA
#define TRIM6_Pin GPIO_PIN_15
#define TRIM6_GPIO_Port GPIOA
#define TRIM5_Pin GPIO_PIN_3
#define TRIM5_GPIO_Port GPIOB
#define TRIM4_Pin GPIO_PIN_4
#define TRIM4_GPIO_Port GPIOB
#define TRIM3_Pin GPIO_PIN_5
#define TRIM3_GPIO_Port GPIOB
#define TRIM2_Pin GPIO_PIN_7
#define TRIM2_GPIO_Port GPIOB
#define TRIM1_Pin GPIO_PIN_8
#define TRIM1_GPIO_Port GPIOB
#define SB_Pin GPIO_PIN_9
#define SB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define EXIT_Pin TRIM2_Pin
#define EXIT_GPIO_Port TRIM2_GPIO_Port
#define ENTER_Pin TRIM3_Pin
#define ENTER_GPIO_Port TRIM3_GPIO_Port
#define DOWN_Pin TRIM4_Pin
#define DOWN_GPIO_Port TRIM4_GPIO_Port
#define UP_Pin TRIM1_Pin
#define UP_GPIO_Port TRIM1_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
