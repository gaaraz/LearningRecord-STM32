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
#define OLED_CS_Pin GPIO_PIN_4
#define OLED_CS_GPIO_Port GPIOA
#define OLED_CLK_Pin GPIO_PIN_5
#define OLED_CLK_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_0
#define OLED_RES_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_1
#define OLED_DC_GPIO_Port GPIOB
#define KEY_LEFT_TOP_Pin GPIO_PIN_2
#define KEY_LEFT_TOP_GPIO_Port GPIOB
#define KEY_RIGHT_TOP_Pin GPIO_PIN_10
#define KEY_RIGHT_TOP_GPIO_Port GPIOB
#define KEY_UP_Pin GPIO_PIN_11
#define KEY_UP_GPIO_Port GPIOB
#define KEY_RIGHT_Pin GPIO_PIN_12
#define KEY_RIGHT_GPIO_Port GPIOB
#define KEY_LEFT_Pin GPIO_PIN_13
#define KEY_LEFT_GPIO_Port GPIOB
#define KEY_DOWN_Pin GPIO_PIN_14
#define KEY_DOWN_GPIO_Port GPIOB
#define POWER_KEY_Pin GPIO_PIN_15
#define POWER_KEY_GPIO_Port GPIOB
#define SI24R1_CS_Pin GPIO_PIN_15
#define SI24R1_CS_GPIO_Port GPIOA
#define SI24R1_IRQ_Pin GPIO_PIN_6
#define SI24R1_IRQ_GPIO_Port GPIOB
#define SI24R1_CE_Pin GPIO_PIN_7
#define SI24R1_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
