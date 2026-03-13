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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* --- HRTIM Outputs --- */
/* PA8  = HRTIM_CHA1 (Timer A Output 1) */
/* PA9  = HRTIM_CHA2 (Timer A Output 2) */
/* PA10 = HRTIM_CHB1 (Timer B Output 1) */
/* PA11 = HRTIM_CHB2 (Timer B Output 2) */

/* --- ADC1 Channels --- */
/* PA0  = ADC1_IN1  (V_HV)  */
/* PA1  = ADC1_IN2  (I_HV)  */
/* PB0  = ADC1_IN15 (V_LV)  */
/* PB1  = ADC1_IN12 (I_LV)  */
/* PB2  = ADC1_IN12 (T_XFMR) - placeholder */

/* --- USART2 --- */
/* PA2  = USART2_TX */
/* PA3  = USART2_RX */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
