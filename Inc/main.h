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
#include "stm32f0xx_hal.h"

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
#define DOSAGE_DRIVER_Pin GPIO_PIN_13
#define DOSAGE_DRIVER_GPIO_Port GPIOC
#define LS_FORM_OPEN_Pin GPIO_PIN_0
#define LS_FORM_OPEN_GPIO_Port GPIOF
#define LS_FORM_CLOSED_Pin GPIO_PIN_1
#define LS_FORM_CLOSED_GPIO_Port GPIOF
#define LS_FORM_PROTECTION_Pin GPIO_PIN_2
#define LS_FORM_PROTECTION_GPIO_Port GPIOC
#define LS_DOSAGE_Pin GPIO_PIN_3
#define LS_DOSAGE_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define CS_MCP_Pin GPIO_PIN_6
#define CS_MCP_GPIO_Port GPIOA
#define CP_DOSAGE_Pin GPIO_PIN_5
#define CP_DOSAGE_GPIO_Port GPIOC
#define CP_SAVE_SETUPS_Pin GPIO_PIN_1
#define CP_SAVE_SETUPS_GPIO_Port GPIOB
#define CP_CNT_RESET_Pin GPIO_PIN_2
#define CP_CNT_RESET_GPIO_Port GPIOB
#define CP_MODE_AUTO_Pin GPIO_PIN_11
#define CP_MODE_AUTO_GPIO_Port GPIOB
#define CP_MODE_MANUAL_Pin GPIO_PIN_12
#define CP_MODE_MANUAL_GPIO_Port GPIOB
#define CP_ENC_BUTTON_Pin GPIO_PIN_15
#define CP_ENC_BUTTON_GPIO_Port GPIOB
#define CP_RETRACTION_Pin GPIO_PIN_6
#define CP_RETRACTION_GPIO_Port GPIOC
#define TEST_Pin GPIO_PIN_7
#define TEST_GPIO_Port GPIOC
#define CP_INJECTION_Pin GPIO_PIN_8
#define CP_INJECTION_GPIO_Port GPIOC
#define CP_OPEN_FORM_Pin GPIO_PIN_9
#define CP_OPEN_FORM_GPIO_Port GPIOC
#define CP_MAIN_PRESSURE_Pin GPIO_PIN_11
#define CP_MAIN_PRESSURE_GPIO_Port GPIOA
#define CP_RESET_Pin GPIO_PIN_12
#define CP_RESET_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RETRACTION_DRIVER_Pin GPIO_PIN_15
#define RETRACTION_DRIVER_GPIO_Port GPIOA
#define OPEN_FORM_DRIVER_Pin GPIO_PIN_10
#define OPEN_FORM_DRIVER_GPIO_Port GPIOC
#define CLOSE_FORM_DRIVER_Pin GPIO_PIN_11
#define CLOSE_FORM_DRIVER_GPIO_Port GPIOC
#define INJECTION_DRIVER_Pin GPIO_PIN_12
#define INJECTION_DRIVER_GPIO_Port GPIOC
#define ALARM_DRIVER_Pin GPIO_PIN_2
#define ALARM_DRIVER_GPIO_Port GPIOD
#define MAIN_PRESSURE_DRIVER_Pin GPIO_PIN_7
#define MAIN_PRESSURE_DRIVER_GPIO_Port GPIOB
#define CP_CLOSE_FORM_Pin GPIO_PIN_8
#define CP_CLOSE_FORM_GPIO_Port GPIOB
#define CS_MAX_Pin GPIO_PIN_9
#define CS_MAX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
