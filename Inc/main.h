/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal.h"

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
//void set_I(int i);
void set_limitA(int a);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOE
#define SIM_RST_Pin GPIO_PIN_1
#define SIM_RST_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_2
#define STOP_GPIO_Port GPIOB
#define RE_Pin GPIO_PIN_8
#define RE_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_11
#define BEEP_GPIO_Port GPIOA
#define KM1_Pin GPIO_PIN_12
#define KM1_GPIO_Port GPIOA
#define FIN_Pin GPIO_PIN_5
#define FIN_GPIO_Port GPIOB
#define TKOUT_Pin GPIO_PIN_6
#define TKOUT_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_7
#define M1_GPIO_Port GPIOB
#define ON_Pin GPIO_PIN_8
#define ON_GPIO_Port GPIOB
#define OFF_Pin GPIO_PIN_9
#define OFF_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
extern uint16_t USART_RX_STA;
extern uint8_t USART_RX_BUF[200];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
