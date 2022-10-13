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
#include "stm32f0xx_hal.h"

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
#define BTN03_Pin GPIO_PIN_4
#define BTN03_GPIO_Port GPIOA
#define BTN02_Pin GPIO_PIN_5
#define BTN02_GPIO_Port GPIOA
#define BTN01_Pin GPIO_PIN_6
#define BTN01_GPIO_Port GPIOA
#define BTN06_Pin GPIO_PIN_10
#define BTN06_GPIO_Port GPIOA
#define BTN05_Pin GPIO_PIN_11
#define BTN05_GPIO_Port GPIOA
#define BTN04_Pin GPIO_PIN_12
#define BTN04_GPIO_Port GPIOA
#define NRF24_SCK_Pin GPIO_PIN_3
#define NRF24_SCK_GPIO_Port GPIOB
#define NRF24_MISO_Pin GPIO_PIN_4
#define NRF24_MISO_GPIO_Port GPIOB
#define NRF24_MOSI_Pin GPIO_PIN_5
#define NRF24_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
