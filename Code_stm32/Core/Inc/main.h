/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define BTV_Pin GPIO_PIN_13
#define BTV_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_0
#define R1_GPIO_Port GPIOA
#define Y1_Pin GPIO_PIN_1
#define Y1_GPIO_Port GPIOA
#define G1_Pin GPIO_PIN_2
#define G1_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_3
#define R2_GPIO_Port GPIOA
#define Y2_Pin GPIO_PIN_4
#define Y2_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_5
#define G2_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_6
#define R3_GPIO_Port GPIOA
#define Y3_Pin GPIO_PIN_7
#define Y3_GPIO_Port GPIOA
#define G3_Pin GPIO_PIN_0
#define G3_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_1
#define R4_GPIO_Port GPIOB
#define Y4_Pin GPIO_PIN_10
#define Y4_GPIO_Port GPIOB
#define G4_Pin GPIO_PIN_11
#define G4_GPIO_Port GPIOB
#define BTDX_Pin GPIO_PIN_12
#define BTDX_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_13
#define D12_GPIO_Port GPIOB
#define C12_Pin GPIO_PIN_14
#define C12_GPIO_Port GPIOB
#define B12_Pin GPIO_PIN_15
#define B12_GPIO_Port GPIOB
#define C11_Pin GPIO_PIN_8
#define C11_GPIO_Port GPIOA
#define A12_Pin GPIO_PIN_9
#define A12_GPIO_Port GPIOA
#define D11_Pin GPIO_PIN_10
#define D11_GPIO_Port GPIOA
#define B11_Pin GPIO_PIN_11
#define B11_GPIO_Port GPIOA
#define A11_Pin GPIO_PIN_12
#define A11_GPIO_Port GPIOA
#define D22_Pin GPIO_PIN_15
#define D22_GPIO_Port GPIOA
#define C22_Pin GPIO_PIN_3
#define C22_GPIO_Port GPIOB
#define B22_Pin GPIO_PIN_4
#define B22_GPIO_Port GPIOB
#define A22_Pin GPIO_PIN_5
#define A22_GPIO_Port GPIOB
#define D21_Pin GPIO_PIN_6
#define D21_GPIO_Port GPIOB
#define C21_Pin GPIO_PIN_7
#define C21_GPIO_Port GPIOB
#define B21_Pin GPIO_PIN_8
#define B21_GPIO_Port GPIOB
#define A21_Pin GPIO_PIN_9
#define A21_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
