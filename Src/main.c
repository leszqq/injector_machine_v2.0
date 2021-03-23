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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio_expander.h"
#include "lcd_by_expander.h"
#include "sev_seg_disp.h"
#include <stdio.h>
#include "encoder.h"
#include "check_macros.h"
#include "button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP_ADDR 0x40                                               // GPIO expander

/* coordinates of values on LCD display */
#define LCD_OPEN_TIME_ROW       0
#define LCD_OPEN_TIME_COL      17
#define LCD_INJECTION_TIME_ROW  1
#define LCD_INJECTION_TIME_COL  17
#define LCD_COOLING_TIME_ROW    2
#define LCD_COOLING_TIME_COL    17

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//struct fsm {
//    int todo;
//};
//
//struct user_menu{
//    GPIO_TypeDef    enc_button_port;
//    uint16_t        enc_button_pin;
//    };
//
//static struct machine {
//    struct fsm_t        fsm;            // finite state machine responsible for handling machine automation cycle
//    struct user_menu    menu;           // user menu for setting periods or maximum periods of cycle stages
//                                        // and cycle counter overflow value.
//}base;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
enum GPIO_expander_status print_lcd_static_text();
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
  lcd_init(&hspi1, MCP_ADDR, CS_MCP_GPIO_Port, CS_MCP_Pin, res_tab);

  sev_seg_init(&hspi1, 1, CS_MAX_GPIO_Port, CS_MAX_Pin);
  sev_seg_write(1);

  encoder_init(&htim1);

  static char text[40] = {0};
  HAL_Delay(100);

  static struct Button but;
  Button_init(&but, CP_ENC_BUTTON_GPIO_Port , CP_ENC_BUTTON_Pin, 30, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t timestamp = HAL_GetTick();
  uint32_t timestamp2 = HAL_GetTick();
  enum sev_seg_digit dig = NONE;

  while (1)
  {
      //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      if(sev_seg_process() != SEV_SEG_OK){
          __NOP();
      }
      if(lcd_process() != GPIO_EXPANDER_OK){
          __NOP();
      }

      Button_process(&but);
      if(Button_been_push(&but))    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

//      if((HAL_GetTick() > timestamp2 + 20)){
//          timestamp2 = HAL_GetTick();
//
//          int16_t trans = encoder_get_transitions();
//          if(trans != 0){
//              snprintf(text, 40, "CNT: %d    ", encoder_get_transitions());
//              lcd_write(text, 0, 0);
//          }
//      }

//      if((HAL_GetTick() > timestamp + 490)){
//          timestamp = HAL_GetTick();
//          dig++;
//          dig %= 5;
//          sev_seg_blink(dig);
//      }
//      HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
}

/* USER CODE BEGIN 4 */
enum GPIO_expander_status print_static_lcd_txt()
{
    enum GPIO_expander_status status;
    status = lcd_write("czas otwarcia:", LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    lcd_write("czas wtrysku:", LCD_INJECTION_TIME_ROW, LCD_INJECTION_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    lcd_write("czas chlodzenia:", LCD_COOLING_TIME_ROW, LCD_COOLING_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    error:
    return status;
}


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
