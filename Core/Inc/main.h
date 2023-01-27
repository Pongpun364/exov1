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
#define emer_pin_Pin GPIO_PIN_4
#define emer_pin_GPIO_Port GPIOE
#define lim1_pin_Pin GPIO_PIN_5
#define lim1_pin_GPIO_Port GPIOE
#define lim2_pin_Pin GPIO_PIN_6
#define lim2_pin_GPIO_Port GPIOE
#define i2c_io_Pin GPIO_PIN_4
#define i2c_io_GPIO_Port GPIOA
#define uart_io_Pin GPIO_PIN_12
#define uart_io_GPIO_Port GPIOE
#define CS_KneeABS_Pin GPIO_PIN_8
#define CS_KneeABS_GPIO_Port GPIOD
#define led_yellow_Pin GPIO_PIN_15
#define led_yellow_GPIO_Port GPIOA
#define canfd1_stb_Pin GPIO_PIN_3
#define canfd1_stb_GPIO_Port GPIOD
#define dip4_Pin GPIO_PIN_4
#define dip4_GPIO_Port GPIOD
#define dip3_Pin GPIO_PIN_5
#define dip3_GPIO_Port GPIOD
#define dip2_Pin GPIO_PIN_6
#define dip2_GPIO_Port GPIOD
#define dip1_Pin GPIO_PIN_7
#define dip1_GPIO_Port GPIOD
#define canfd2_stb_Pin GPIO_PIN_7
#define canfd2_stb_GPIO_Port GPIOB
#define led_red_Pin GPIO_PIN_9
#define led_red_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
