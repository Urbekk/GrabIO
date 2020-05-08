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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define enkoder1_Pin GPIO_PIN_0
#define enkoder1_GPIO_Port GPIOA
#define enkoder2_Pin GPIO_PIN_1
#define enkoder2_GPIO_Port GPIOA
#define chwytak_Pin GPIO_PIN_7
#define chwytak_GPIO_Port GPIOA
#define kierunkiP1_Pin GPIO_PIN_4
#define kierunkiP1_GPIO_Port GPIOC
#define kierunkiP2_Pin GPIO_PIN_5
#define kierunkiP2_GPIO_Port GPIOC
#define silnikPrawy_Pin GPIO_PIN_11
#define silnikPrawy_GPIO_Port GPIOE
#define silnikLewy_Pin GPIO_PIN_14
#define silnikLewy_GPIO_Port GPIOE
#define kierunkiL1_Pin GPIO_PIN_15
#define kierunkiL1_GPIO_Port GPIOE
#define kierunkiL2_Pin GPIO_PIN_10
#define kierunkiL2_GPIO_Port GPIOB
#define przegub2_Pin GPIO_PIN_12
#define przegub2_GPIO_Port GPIOD
#define przegub1_Pin GPIO_PIN_13
#define przegub1_GPIO_Port GPIOD
#define podstawa2_Pin GPIO_PIN_14
#define podstawa2_GPIO_Port GPIOD
#define podstawa1_Pin GPIO_PIN_15
#define podstawa1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
