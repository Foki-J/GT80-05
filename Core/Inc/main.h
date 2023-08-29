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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RS485_handle(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_RESET_Pin GPIO_PIN_0
#define ETH_RESET_GPIO_Port GPIOC
#define THERMAL0_Pin GPIO_PIN_4
#define THERMAL0_GPIO_Port GPIOA
#define THERMAL1_Pin GPIO_PIN_5
#define THERMAL1_GPIO_Port GPIOA
#define LIGHT_FRONT_Pin GPIO_PIN_1
#define LIGHT_FRONT_GPIO_Port GPIOD
#define LIGHT_BACK_Pin GPIO_PIN_3
#define LIGHT_BACK_GPIO_Port GPIOD
#define EN485_Pin GPIO_PIN_11
#define EN485_GPIO_Port GPIOA
#define CAMERA_TRIG_Pin GPIO_PIN_11
#define CAMERA_TRIG_GPIO_Port GPIOC
#define SPEAKER_Pin GPIO_PIN_4
#define SPEAKER_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */
extern uint8_t rxsize;
extern unsigned char USART1_RxBuft[40];//接受处理寄存器
extern unsigned char USART1_FLME;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
