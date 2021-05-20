/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_EN_MCU_Pin GPIO_PIN_1
#define POWER_EN_MCU_GPIO_Port GPIOA
#define SX1302_RESET_MCU_Pin GPIO_PIN_2
#define SX1302_RESET_MCU_GPIO_Port GPIOA
#define SX1261_BUSY_Pin GPIO_PIN_3
#define SX1261_BUSY_GPIO_Port GPIOA
#define SX1302_CS_Pin GPIO_PIN_4
#define SX1302_CS_GPIO_Port GPIOA
#define SX1261_CS_Pin GPIO_PIN_0
#define SX1261_CS_GPIO_Port GPIOB
#define SX1261_DIO1_Pin GPIO_PIN_1
#define SX1261_DIO1_GPIO_Port GPIOB
#define SX1261_RESET_Pin GPIO_PIN_8
#define SX1261_RESET_GPIO_Port GPIOA
#define PPS_Pin GPIO_PIN_4
#define PPS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
