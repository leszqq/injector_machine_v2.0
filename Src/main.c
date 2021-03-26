/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio_expander.h"
#include "lcd_by_expander.h"
#include "sev_seg_disp.h"
#include <stdio.h>
#include "encoder.h"
#include "check_macros.h"
#include "counter.h"
#include "time_table.h"
#include "menu.h"
#include "status_tab.h"
#include "check_macros.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP_ADDR 0x40                                               // GPIO expander

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* singleton machine instance */
static struct Machine {
    struct Fsm                  fsm;            // finite state machine responsible for handling machine automation cycle
    struct Time_table           cycle_times;    // contains periods for time dependent machine cycle steps
    struct Counter              counter;
} base;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(500);

  /* pin mapping for GPIO_Expander nad LCD */
  struct Resolve_table res_tab = {
          .RW = GP5,
          .RS = GP6,
          .E = GP4,
          .D4 = GP3,
          .D5 = GP2,
          .D6 = GP1,
          .D7 = GP0
  };


  enum EEPROM_status eeprom_stat = EEPROM_init(&hi2c2);
  DBG_LOG("EEPROM init: %d", eeprom_stat);

  eeprom_stat = EEPROM_read_setups(&base.cycle_times, &base.counter.max_value);
  DBG_LOG("EEPROM read: %d", eeprom_stat);

  /* set default values if eeprom read failed */
  if(eeprom_stat != EEPROM_OK){
      base.counter.max_value = 63;
      base.cycle_times.open_time = 2000;
      base.cycle_times.injection_time = 7000;
      base.cycle_times.cooling_time = 10000;
  }

  /* initialzie displays */
  enum GPIO_expander_status gpio_exp_stat = Lcd_init(&hspi1, MCP_ADDR, CS_MCP_GPIO_Port, CS_MCP_Pin, res_tab);
  DBG_LOG("Lcd_init: %d", gpio_exp_stat);

  enum sev_seg_status sev_seg_stat = Sev_seg_init(&hspi1, 1, CS_MAX_GPIO_Port, CS_MAX_Pin);
  DBG_LOG("Sev_seg_init: %d", sev_seg_stat);

  Encoder_init(&htim1);

  /* initialize user menu */
  Menu_init(&base.cycle_times, &base.fsm.state, &base.counter);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint32_t timestamp = HAL_GetTick();
  while (1)
  {
      //if(Button_been_push(2)) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      /* machine process here */
      // TODO

      /* handle user menu interface */
      Menu_process();


      if(HAL_GetTick() > timestamp + 500){
          timestamp = HAL_GetTick();
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  //while(1){};

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
