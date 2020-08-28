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
static uint8_t t=0xF0;
static uint8_t* f=&t;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MINFEED 1300000
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define FEEDSPD_Pin GPIO_PIN_0
#define FEEDSPD_GPIO_Port GPIOC
#define MOTIONSPD_Pin GPIO_PIN_1
#define MOTIONSPD_GPIO_Port GPIOC
#define FENA_Pin GPIO_PIN_0
#define FENA_GPIO_Port GPIOA
#define FEEDPULSE_Pin GPIO_PIN_1
#define FEEDPULSE_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define FDIR_Pin GPIO_PIN_4
#define FDIR_GPIO_Port GPIOA
#define PLC_SCK_Pin GPIO_PIN_5
#define PLC_SCK_GPIO_Port GPIOA
#define PLC_MISO_Pin GPIO_PIN_6
#define PLC_MISO_GPIO_Port GPIOA
#define PLC_MOSI_Pin GPIO_PIN_7
#define PLC_MOSI_GPIO_Port GPIOA
#define H1_RED_ALARM_Pin GPIO_PIN_5
#define H1_RED_ALARM_GPIO_Port GPIOC
#define SPI5_ILI9225_CLK_Pin GPIO_PIN_0
#define SPI5_ILI9225_CLK_GPIO_Port GPIOB
#define H2_GREEN_READY_Pin GPIO_PIN_6
#define H2_GREEN_READY_GPIO_Port GPIOC
#define H1_RED_ALARMC8_Pin GPIO_PIN_8
#define H1_RED_ALARMC8_GPIO_Port GPIOC
#define ROTSTENA_Pin GPIO_PIN_8
#define ROTSTENA_GPIO_Port GPIOA
#define ILI9225_RST_Pin GPIO_PIN_9
#define ILI9225_RST_GPIO_Port GPIOA
#define SPI5_ILI9225_SDA_Pin GPIO_PIN_10
#define SPI5_ILI9225_SDA_GPIO_Port GPIOA
#define MODBUS_TX_Pin GPIO_PIN_11
#define MODBUS_TX_GPIO_Port GPIOA
#define MODBUS_RX_Pin GPIO_PIN_12
#define MODBUS_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ROTSTPULSE_Pin GPIO_PIN_15
#define ROTSTPULSE_GPIO_Port GPIOA
#define ILI9225_RS_Pin GPIO_PIN_11
#define ILI9225_RS_GPIO_Port GPIOC
#define SERVOPULSE_Pin GPIO_PIN_4
#define SERVOPULSE_GPIO_Port GPIOB
#define SIGN_Pin GPIO_PIN_5
#define SIGN_GPIO_Port GPIOB
#define ILI9225_CS_Pin GPIO_PIN_7
#define ILI9225_CS_GPIO_Port GPIOB
#define ROTSTDIR_Pin GPIO_PIN_9
#define ROTSTDIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void RELAY_Handler(uint8_t* iData);
void PLC_Handler(void);
uint8_t* CURRENT_LIMITER_Handler(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
