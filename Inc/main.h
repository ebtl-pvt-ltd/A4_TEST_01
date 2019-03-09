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
#include "stm32f3xx_hal.h"

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
#define ESP_RST_Pin GPIO_PIN_6
#define ESP_RST_GPIO_Port GPIOE
#define RGB_OUT_Pin GPIO_PIN_9
#define RGB_OUT_GPIO_Port GPIOF
#define RL_1_Pin GPIO_PIN_7
#define RL_1_GPIO_Port GPIOE
#define RL_2_Pin GPIO_PIN_8
#define RL_2_GPIO_Port GPIOE
#define RL_3_Pin GPIO_PIN_9
#define RL_3_GPIO_Port GPIOE
#define RL_4_Pin GPIO_PIN_10
#define RL_4_GPIO_Port GPIOE
#define RL_5_Pin GPIO_PIN_11
#define RL_5_GPIO_Port GPIOE
#define RL_6_Pin GPIO_PIN_12
#define RL_6_GPIO_Port GPIOE
#define RL_7_Pin GPIO_PIN_13
#define RL_7_GPIO_Port GPIOE
#define RL_8_Pin GPIO_PIN_14
#define RL_8_GPIO_Port GPIOE
#define RL_9_Pin GPIO_PIN_15
#define RL_9_GPIO_Port GPIOE
#define M_Sense_Pin GPIO_PIN_11
#define M_Sense_GPIO_Port GPIOD
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_10
#define SCK_GPIO_Port GPIOC
#define MISO_Pin GPIO_PIN_11
#define MISO_GPIO_Port GPIOC
#define MOSI_Pin GPIO_PIN_12
#define MOSI_GPIO_Port GPIOC
#define TFT_A0_Pin GPIO_PIN_0
#define TFT_A0_GPIO_Port GPIOD
#define TFT_RST_Pin GPIO_PIN_1
#define TFT_RST_GPIO_Port GPIOD
#define TFT_CS_Pin GPIO_PIN_2
#define TFT_CS_GPIO_Port GPIOD
#define SD_DET_Pin GPIO_PIN_3
#define SD_DET_GPIO_Port GPIOD
#define IR_IN_Pin GPIO_PIN_8
#define IR_IN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
