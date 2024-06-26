/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define SERVO_MOTOR_IN2_Pin GPIO_PIN_9
#define SERVO_MOTOR_IN2_GPIO_Port GPIOB
#define NRF24_CSN_Pin GPIO_PIN_2
#define NRF24_CSN_GPIO_Port GPIOA
#define CAR_BREAKLIGHT_Pin GPIO_PIN_3
#define CAR_BREAKLIGHT_GPIO_Port GPIOA
#define NRF24_SPI1_CS_Pin GPIO_PIN_4
#define NRF24_SPI1_CS_GPIO_Port GPIOA
#define NRF24_SPI1_SCK_Pin GPIO_PIN_5
#define NRF24_SPI1_SCK_GPIO_Port GPIOA
#define NRF24_SPI1_MISO_Pin GPIO_PIN_6
#define NRF24_SPI1_MISO_GPIO_Port GPIOA
#define NRF24_SPI1_MOSI_Pin GPIO_PIN_7
#define NRF24_SPI1_MOSI_GPIO_Port GPIOA
#define MOTOR_CONTROL_IN1_Pin GPIO_PIN_0
#define MOTOR_CONTROL_IN1_GPIO_Port GPIOB
#define NRF24_IRQ_Pin GPIO_PIN_11
#define NRF24_IRQ_GPIO_Port GPIOA
#define NRF24_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define CAR_HEADLIGHT_Pin GPIO_PIN_12
#define CAR_HEADLIGHT_GPIO_Port GPIOA
#define MOTOR_CONTROL_IN0_Pin GPIO_PIN_4
#define MOTOR_CONTROL_IN0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
