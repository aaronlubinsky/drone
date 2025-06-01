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
#define UART_TX_TO_BT_Pin GPIO_PIN_2
#define UART_TX_TO_BT_GPIO_Port GPIOA
#define UART_RX_FROM_BT_Pin GPIO_PIN_3
#define UART_RX_FROM_BT_GPIO_Port GPIOA
#define US_ECHO_Pin GPIO_PIN_12
#define US_ECHO_GPIO_Port GPIOB
#define US_TRIG_Pin GPIO_PIN_13
#define US_TRIG_GPIO_Port GPIOB
#define TX_TO_ST_Pin GPIO_PIN_9
#define TX_TO_ST_GPIO_Port GPIOA
#define RX_FROM_ST_Pin GPIO_PIN_10
#define RX_FROM_ST_GPIO_Port GPIOA
#define BNO_SCL_Pin GPIO_PIN_6
#define BNO_SCL_GPIO_Port GPIOB
#define BNO_SDA_Pin GPIO_PIN_7
#define BNO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
