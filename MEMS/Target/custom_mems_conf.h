/**
 ******************************************************************************
 * @file    custom_mems_conf.h
 * @author  MEMS Application Team
 * @version V7.0.0
 * @date    24-June-2019
 * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_MEMS_CONF_H__
#define __CUSTOM_MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_CUSTOM_ENV_SENSOR_STTS751_0           1U

#define CUSTOM_STTS751_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_STTS751_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_STTS751_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_STTS751_0_I2C_WriteReg BSP_I2C1_WriteReg

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_MEMS_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
