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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_nucleo.h"
#include <stdio.h>

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
#define SDCARD_MOSI_Pin GPIO_PIN_1
#define SDCARD_MOSI_GPIO_Port GPIOC
#define SDCARD_MISO_Pin GPIO_PIN_2
#define SDCARD_MISO_GPIO_Port GPIOC
#define IRQ_WINC_PIN_Pin GPIO_PIN_4
#define IRQ_WINC_PIN_GPIO_Port GPIOC
#define CS_WINC_Pin GPIO_PIN_5
#define CS_WINC_GPIO_Port GPIOC
#define RESET_WINC_Pin GPIO_PIN_0
#define RESET_WINC_GPIO_Port GPIOB
#define CHIP_EN_WINC_Pin GPIO_PIN_1
#define CHIP_EN_WINC_GPIO_Port GPIOB
#define SDCARD_SCK_Pin GPIO_PIN_10
#define SDCARD_SCK_GPIO_Port GPIOB
#define DISPLAY_RX_Pin GPIO_PIN_7
#define DISPLAY_RX_GPIO_Port GPIOC
#define DISPLAY_TX_Pin GPIO_PIN_8
#define DISPLAY_TX_GPIO_Port GPIOA
#define SDCARD_CS_Pin GPIO_PIN_8
#define SDCARD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
