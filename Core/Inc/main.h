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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
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
#define RS485EN_Pin GPIO_PIN_13
#define RS485EN_GPIO_Port GPIOC
#define CURRA_Pin GPIO_PIN_0
#define CURRA_GPIO_Port GPIOA
#define CURRB_Pin GPIO_PIN_1
#define CURRB_GPIO_Port GPIOA
#define VCCSENSE_Pin GPIO_PIN_2
#define VCCSENSE_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_3
#define ENA_GPIO_Port GPIOA
#define ENB_Pin GPIO_PIN_4
#define ENB_GPIO_Port GPIOA
#define ENC_Pin GPIO_PIN_5
#define ENC_GPIO_Port GPIOA
#define PULSEA_Pin GPIO_PIN_6
#define PULSEA_GPIO_Port GPIOA
#define PULSEB_Pin GPIO_PIN_7
#define PULSEB_GPIO_Port GPIOA
#define PULSEC_Pin GPIO_PIN_0
#define PULSEC_GPIO_Port GPIOB
#define TEMP_Pin GPIO_PIN_1
#define TEMP_GPIO_Port GPIOB
#define AS5147_CS_Pin GPIO_PIN_8
#define AS5147_CS_GPIO_Port GPIOA
#define SWD_SWDIO_Pin GPIO_PIN_13
#define SWD_SWDIO_GPIO_Port GPIOA
#define SWD_SWCLK_Pin GPIO_PIN_14
#define SWD_SWCLK_GPIO_Port GPIOA
#define AS5147_SCK_Pin GPIO_PIN_3
#define AS5147_SCK_GPIO_Port GPIOB
#define AS5147_MISO_Pin GPIO_PIN_4
#define AS5147_MISO_GPIO_Port GPIOB
#define AS5147_MOSI_Pin GPIO_PIN_5
#define AS5147_MOSI_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
