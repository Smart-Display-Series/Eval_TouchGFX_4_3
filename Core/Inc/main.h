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
#include "stm32f7xx_hal.h"

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
#define Flash_WP_Pin GPIO_PIN_4
#define Flash_WP_GPIO_Port GPIOE
#define LCD_BL_CTRL_Pin GPIO_PIN_5
#define LCD_BL_CTRL_GPIO_Port GPIOE
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define External_ADC_IN_Pin GPIO_PIN_1
#define External_ADC_IN_GPIO_Port GPIOC
#define USART3_DE_Pin GPIO_PIN_14
#define USART3_DE_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_15
#define LCD_RST_GPIO_Port GPIOB
#define LTDC_DISP_Pin GPIO_PIN_2
#define LTDC_DISP_GPIO_Port GPIOG
#define CTP_RST_Pin GPIO_PIN_8
#define CTP_RST_GPIO_Port GPIOC
#define CTP_SDA_Pin GPIO_PIN_9
#define CTP_SDA_GPIO_Port GPIOC
#define CTP_SCL_Pin GPIO_PIN_8
#define CTP_SCL_GPIO_Port GPIOA
#define CTP_INT_Pin GPIO_PIN_10
#define CTP_INT_GPIO_Port GPIOA
#define GPIO_OUT_BEEP_Pin GPIO_PIN_13
#define GPIO_OUT_BEEP_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
