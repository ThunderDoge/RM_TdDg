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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NC_Pin GPIO_PIN_2
#define NC_GPIO_Port GPIOA
#define DBUS_Pin GPIO_PIN_3
#define DBUS_GPIO_Port GPIOA
#define ICM_CS_Pin GPIO_PIN_4
#define ICM_CS_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_4
#define OLED_CS_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_5
#define OLED_DC_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_0
#define OLED_RES_GPIO_Port GPIOB
#define LAZER_Pin GPIO_PIN_1
#define LAZER_GPIO_Port GPIOB
#define VISION_TX_Pin GPIO_PIN_10
#define VISION_TX_GPIO_Port GPIOB
#define VISION_RX_Pin GPIO_PIN_11
#define VISION_RX_GPIO_Port GPIOB
#define DBUSA10_Pin GPIO_PIN_10
#define DBUSA10_GPIO_Port GPIOA
#define SUPERIOR_TX_Pin GPIO_PIN_12
#define SUPERIOR_TX_GPIO_Port GPIOC
#define SUPERIOR_RX_Pin GPIO_PIN_2
#define SUPERIOR_RX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
