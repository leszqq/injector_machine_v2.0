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
enum mode {
    MODE_MANUAL,
    MODE_SEMI_AUTO,
    MODE_AUTO
};

enum valve {
    VALVE_
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP_ADDR 0x40                                               // GPIO expander
#define MESSAGE_ROW                 3
#define MESSAGE_COL                 0
#define AUX_TAB_LEN                 21

#define WATCHDOG_THRESHOLD          40000                           // maximum period of cycle phase

#define WATCHDOG_MS_TIMER           htim14
#define AUX_PHASE_MS_TIMER          htim16
#define CYCLE_CNT_ALARM_TIMER       htim17

#define CYCLE_CNT_ALARM_TIME        3000                            // Duration of sound indication
                                                                    // of cycle counter overflow
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* singleton machine instance */
static struct Machine {
    struct Fsm                  fsm;                    // finite state machine responsible for handling machine automation cycle
    struct Time_table           cycle_times;            // contains periods for time dependent machine cycle steps
    struct Counter              counter;
    enum mode                   operating_mode;
    uint8_t                     aux_tab[AUX_TAB_LEN];   // auxiliary tab
} base;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static bool is_control_button_push(GPIO_TypeDef* GPIOx, uint16_t pin);
static bool is_form_closed();
static bool is_form_open();
static bool is_dosage_finish();
static bool form_protection_activated();

static enum mode get_operation_mode();
static void manual_mode_process();
static void auto_mode_process();

static void turn_off_all_valves();
static void set_alarm(bool alarm_on);

/* turn off all valves, set alarm on, change fsm state to ERR, and print message on lcd */
static void error(const char* message);
static void print_message(const char* message);

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
  /* wait for supply voltage stabilization */
  HAL_Delay(500);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* initializa base struct */
  base.operating_mode = MODE_MANUAL;
  base.fsm.state = STOP;


  /* pin mapping for GPIO_Expander and LCD */
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

  /* initialize displays */
  enum GPIO_expander_status gpio_exp_stat = Lcd_init(&hspi1, MCP_ADDR, CS_MCP_GPIO_Port, CS_MCP_Pin, res_tab);
  DBG_LOG("Lcd_init: %d", gpio_exp_stat);
  HAL_Delay(100);

  enum sev_seg_status sev_seg_stat = Sev_seg_init(&hspi1, 1, CS_MAX_GPIO_Port, CS_MAX_Pin);
  DBG_LOG("Sev_seg_init: %d", sev_seg_stat);

  Encoder_init(&htim1);

  /* initialize user menu */
  Menu_init(&base.cycle_times, &base.fsm.state, &base.counter);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint32_t timestamp = HAL_GetTick();
  enum mode new_mode;


  while (1)
  {
      /* check if user changed operating mode */
      new_mode = get_operation_mode();
      if(base.operating_mode != new_mode){
          /* for transitions from manual mode to auto or semi-auto mode set fsm state STOP.
           * Turn off all driver valves in case user kept pushing any control button when
           * switching mode selection switch.*/
          if(base.operating_mode == MODE_MANUAL && new_mode != MODE_MANUAL) {
              turn_off_all_valves();
              base.operating_mode = new_mode;
              base.fsm.state = STOP;
          }
      }

      /* machine process */
      if(base.operating_mode == MODE_AUTO || base.operating_mode == MODE_SEMI_AUTO){
          auto_mode_process();
      } else {
          manual_mode_process();
      }

      /* handle user menu interface */
      Menu_process();

      /* debug diode blink */
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
bool is_control_button_push(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(GPIOx, pin) == GPIO_PIN_RESET);
}

bool is_form_closed()
{
    return (HAL_GPIO_ReadPin(LS_FORM_CLOSED_GPIO_Port, LS_FORM_CLOSED_Pin) == GPIO_PIN_RESET);
}

bool is_form_open()
{
    return (HAL_GPIO_ReadPin(LS_FORM_OPEN_GPIO_Port, LS_FORM_OPEN_Pin) == GPIO_PIN_RESET);
}

bool is_dosage_finish()
{
    return (HAL_GPIO_ReadPin(LS_DOSAGE_GPIO_Port, LS_DOSAGE_Pin) == GPIO_PIN_SET);
}

bool form_protection_activated()
{
    return HAL_GPIO_ReadPin(LS_FORM_PROTECTION_GPIO_Port, LS_FORM_PROTECTION_Pin) == GPIO_PIN_RESET;
}
enum mode get_operation_mode()
{
    if(HAL_GPIO_ReadPin(CP_MODE_AUTO_GPIO_Port, CP_MODE_AUTO_Pin) == GPIO_PIN_RESET){
        return MODE_AUTO;
    } else if(HAL_GPIO_ReadPin(CP_MODE_MANUAL_GPIO_Port, CP_MODE_MANUAL_Pin) == GPIO_PIN_RESET){
        return MODE_MANUAL;
    } else {
        return MODE_SEMI_AUTO;
    }
}

void manual_mode_process()
{
    if(base.fsm.state == ERR){
        /* wait for user to push error reset button, then return to manual mode*/
        if(is_control_button_push(CP_RESET_GPIO_Port, CP_RESET_Pin)){
            set_alarm(false);
            base.fsm.state = STOP;
            print_message("");              // clear message field
        }
    } else {
        /* normal manual mode operation */

        /* form protection */
        if(form_protection_activated()){
            turn_off_all_valves();
            set_alarm(true);
            base.fsm.state = ERR;
            return;
        }

        /* MAIN PRESSURE BUTTON */
        /* Activate main pressure valve if main pressure button push and dosage button released
         * (two way valve) */
        if(is_control_button_push(CP_MAIN_PRESSURE_GPIO_Port, CP_MAIN_PRESSURE_Pin)
        && !is_control_button_push(CP_DOSAGE_GPIO_Port, CP_DOSAGE_Pin)){
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_SET);

            /* Deactivate main pressure valve if main pressure button, injection and retraction
             * button released */
        } else if (!is_control_button_push(CP_INJECTION_GPIO_Port, CP_INJECTION_Pin)
        && !is_control_button_push(CP_RETRACTION_GPIO_Port, CP_RETRACTION_Pin)) {
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_RESET);
        }

        /* FROM CLOSING BUTTON */
        /* Activate form closing valve if main pressure, close form buttons push, open form button
         * released and form is not closed yet */
        if(is_control_button_push(CP_CLOSE_FORM_GPIO_Port, CP_CLOSE_FORM_Pin)
        && is_control_button_push(CP_MAIN_PRESSURE_GPIO_Port, CP_MAIN_PRESSURE_Pin)
        && !is_control_button_push(CP_OPEN_FORM_GPIO_Port, CP_OPEN_FORM_Pin)
        && !is_form_closed()) {
            HAL_GPIO_WritePin(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin, GPIO_PIN_SET);

            /* else, deactivate form closing valve*/
        } else {
            HAL_GPIO_WritePin(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin, GPIO_PIN_RESET);
        }

        /* FORM OPEN BUTTON */
        /* Activate form open valve if main pressure, open form buttons push, close form button
         * released and form is not open yet*/
        if(is_control_button_push(CP_MAIN_PRESSURE_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin)
        && is_control_button_push(CP_OPEN_FORM_GPIO_Port, CP_OPEN_FORM_Pin)
        && !is_control_button_push(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin)
        && !is_form_open()) {
            HAL_GPIO_WritePin(OPEN_FORM_DRIVER_GPIO_Port, OPEN_FORM_DRIVER_Pin, GPIO_PIN_SET);

            /* else, deactivate form open valve */
        } else {
            HAL_GPIO_WritePin(OPEN_FORM_DRIVER_GPIO_Port, OPEN_FORM_DRIVER_Pin, GPIO_PIN_RESET);
        }

        /* INJECTION BUTTON */
        /* Activate injection and main pressure valve if injection button is push and retraction
         * button is released.*/
        if(is_control_button_push(CP_INJECTION_GPIO_Port, CP_INJECTION_Pin)
        && !is_control_button_push(CP_RETRACTION_GPIO_Port, CP_RETRACTION_Pin)) {
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(INJECTION_DRIVER_GPIO_Port, INJECTION_DRIVER_Pin, GPIO_PIN_SET);

            /* else, deactivate injection valve */
        } else {
            HAL_GPIO_WritePin(INJECTION_DRIVER_GPIO_Port, INJECTION_DRIVER_Pin, GPIO_PIN_RESET);
        }

        /* RETRACTION BUTTON */
        /* Activate retraction and main pressure valve if retraction button is push and injection button is released */
        if(is_control_button_push(CP_RETRACTION_GPIO_Port, CP_RETRACTION_Pin)
        && !is_control_button_push(CP_INJECTION_GPIO_Port, CP_INJECTION_Pin)) {
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RETRACTION_DRIVER_GPIO_Port, RETRACTION_DRIVER_Pin, GPIO_PIN_SET);

            /* else, deactivate retraction valve */
        } else {
            HAL_GPIO_WritePin(RETRACTION_DRIVER_GPIO_Port, RETRACTION_DRIVER_Pin, GPIO_PIN_RESET);
        }

        /* DOSAGE BUTTON */
        /* If dosage button push, main pressure button released and
         * dosage finish limit switch not active, then activate dosage valve and deactivate
         * main pressure valve. */
        if(is_control_button_push(CP_DOSAGE_GPIO_Port, CP_DOSAGE_Pin)
        && !is_control_button_push(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin)
        && !is_dosage_finish()) {
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_SET);

            /* else, deactivate dosage valve */
        } else {
            HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_RESET);
        }
    }
}

void auto_mode_process()
{
    /* fsm realization for auto and semi-auto mode */
    switch(base.fsm.state){
    case STOP:
        /* start cycle on main pressure and close form buttons push */
        if(is_control_button_push(CP_MAIN_PRESSURE_GPIO_Port, CP_MAIN_PRESSURE_Pin)
        && is_control_button_push(CP_CLOSE_FORM_GPIO_Port, CP_CLOSE_FORM_Pin)) {
            /* assert that machine can start cycle, indeed that form is open, material is dosaged
             * and form protection have not been activated */
            if(!is_form_open()) {
                error("Forma nieotwarta");
            } else if (!is_dosage_finish()){
                error("Tworzywo niepobrane");
            } else if (form_protection_activated()) {
                error("Zabezpieczenie formy");
            } else {
                /* drive vavles to close form */
                HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin, GPIO_PIN_SET);

                /* Start and reset watchdog */
                HAL_TIM_Base_Start(&WATCHDOG_MS_TIMER);
                WATCHDOG_MS_TIMER.Instance->CNT = 0;

                /* increment cycle counter */
                base.counter.value++;

                print_message("zamykanie formy");
                DBG_LOG("STOP->FORM_CLOSING");
                base.fsm.state = FORM_CLOSING;
            }
        }
        break;
    case PAUSE:
        /* Check if pause time elapsed */
        if(AUX_PHASE_MS_TIMER.Instance->CNT >= base.cycle_times.open_time){
            /* turn off aux timer, reset watchdog */
            HAL_TIM_Base_Stop(&AUX_PHASE_MS_TIMER);
            WATCHDOG_MS_TIMER.Instance->CNT = 0;

            print_message("zamykanie formy");
            base.fsm.state = FORM_CLOSING;
            DBG_LOG("PAUSE->FORM_CLOSING");
        }
        break;
    case FORM_CLOSING:
        /* Scan for form protection activated */
        if(form_protection_activated())     error("zabezpieczenie formy");
        /* If form already closed, start injection phase.*/
        if(is_form_closed()) {
            /* turn off valve responsible for closing form */
            HAL_GPIO_WritePin(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin, GPIO_PIN_RESET);
            /* enable injection valve */
            HAL_GPIO_WritePin(INJECTION_DRIVER_GPIO_Port, INJECTION_DRIVER_Pin, GPIO_PIN_SET);
            /*reset watchdog */
            WATCHDOG_MS_TIMER.Instance->CNT = 0;
            /* start injection period counting */
            HAL_TIM_Base_Start(&AUX_PHASE_MS_TIMER);
            AUX_PHASE_MS_TIMER.Instance->CNT = 0;

            print_message("zamykanie formy");

            base.fsm.state = INJECTION;
            DBG_LOG("FORM_CLOSING->INJECTION");
        }
        break;
    case INJECTION:
        /* check if injection time passed.*/
        if(AUX_PHASE_MS_TIMER.Instance->CNT >= base.cycle_times.injection_time){
            /* disable injection and main pressure valves, start dosage, start counting dosage time,
             * reset watchdog */
            HAL_GPIO_WritePin(INJECTION_DRIVER_GPIO_Port, INJECTION_DRIVER_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_RESET);
            /* if material not already in chamber, enable dosage valve */
            if(!is_dosage_finish()) HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_SET);

            AUX_PHASE_MS_TIMER.Instance->CNT = 0;
            WATCHDOG_MS_TIMER.Instance->CNT = 0;

            /* if cycle counter equal to max cycle counter value, turn on sound alarm for 3 seconds */
            if(base.counter.value == base.counter.max_value){
                set_alarm(true);
                HAL_TIM_Base_Start(&CYCLE_CNT_ALARM_TIMER);
                CYCLE_CNT_ALARM_TIMER.Instance->CNT = 0;
            }

            print_message("chlodzenie");
            base.fsm.state = COOLING;
            DBG_LOG("INJECTION->COOLING");
        }
        break;
    case COOLING:
        /* disable dosage valve if enough material accumulated in chamber */
        if(is_dosage_finish())  HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_RESET);
        /* check if cooling time passed. */
        if(AUX_PHASE_MS_TIMER.Instance->CNT >= base.cycle_times.cooling_time) {
            HAL_TIM_Base_Stop(&AUX_PHASE_MS_TIMER);
            /* Call error if material still not accumulated. */
            if(!is_dosage_finish()){
                error("przekr. czas dozow.");
                break;
            }
            /* open form, reset watchdog. */
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OPEN_FORM_DRIVER_GPIO_Port, OPEN_FORM_DRIVER_Pin, GPIO_PIN_SET);


            WATCHDOG_MS_TIMER.Instance->CNT = 0;
            print_message("otwieranie formy");
            base.fsm.state = FORM_OPENING;
            DBG_LOG("COOLING->FORM_OPENING");
        }
        break;
    case FORM_OPENING:
        /* check if form is already open. */
        if(is_form_open()) {
            /* turn off form opening valve, reset watchdog */
            HAL_GPIO_WritePin(OPEN_FORM_DRIVER_GPIO_Port, OPEN_FORM_DRIVER_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_RESET);

            WATCHDOG_MS_TIMER.Instance->CNT = 0;

            if(base.operating_mode == MODE_AUTO) {
                /*start counting pause time */
                HAL_TIM_Base_Start(&AUX_PHASE_MS_TIMER);
                AUX_PHASE_MS_TIMER.Instance->CNT = 0;

                print_message("pauza");
                base.fsm.state = PAUSE;
                DBG_LOG("INJECTION->PAUSE");
            } else {
                /* semi - auto mode */
                print_message("stop");
                base.fsm.state = STOP;
                DBG_LOG("FORM_OPENING->STOP");
            }
        }
        break;
    case ERR:
        /* wait for user to push error reset button, then return to STOP state */
        if(is_control_button_push(CP_RESET_GPIO_Port, CP_RESET_Pin)) {
            set_alarm(false);
            base.fsm.state = STOP;
            print_message("");                   // clear message field
        }
        break;
    default:
        WARN(0, "Shouldn't get here");
        /* state recovery */
        base.fsm.state = STOP;
        break;
    }

    /* turn off sound signal at cycle counter overflow after CYCLE_CNT_ALARM_TIME seconds */
    if(base.counter.value == base.counter.max_value
    && CYCLE_CNT_ALARM_TIMER.Instance->CNT >= CYCLE_CNT_ALARM_TIME
    && base.fsm.state != ERR) {
        set_alarm(false);
        CYCLE_CNT_ALARM_TIMER.Instance->CNT = 0;
        HAL_TIM_Base_Start(&CYCLE_CNT_ALARM_TIMER);
        DBG_LOG("CNT OVFLW sound off");
    }

    /* check for watchdog overflow */
    if(base.fsm.state != STOP || base.fsm.state != ERR){
        if(WATCHDOG_MS_TIMER.Instance->CNT > WATCHDOG_THRESHOLD) {
            error("przekr. czas cyklu");

            base.fsm.state = ERR;
            DBG_LOG("Watchdog overflow.")
            DBG_LOG("fsm_state:%d->ERR", base.fsm.state);
        }
    }
}

void turn_off_all_valves()
{
    HAL_GPIO_WritePin(MAIN_PRESSURE_DRIVER_GPIO_Port, MAIN_PRESSURE_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLOSE_FORM_DRIVER_GPIO_Port, CLOSE_FORM_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OPEN_FORM_DRIVER_GPIO_Port, OPEN_FORM_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INJECTION_DRIVER_GPIO_Port, INJECTION_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RETRACTION_DRIVER_GPIO_Port, RETRACTION_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DOSAGE_DRIVER_GPIO_Port, DOSAGE_DRIVER_Pin, GPIO_PIN_RESET);
}

void set_alarm(bool alarm_on)
{
    if(alarm_on){
        HAL_GPIO_WritePin(ALARM_DRIVER_GPIO_Port, ALARM_DRIVER_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(ALARM_DRIVER_GPIO_Port, ALARM_DRIVER_Pin, GPIO_PIN_RESET);
    }
}

/* turn off all valves, set alarm on, change fsm state to ERR, and print message on lcd */
static void error(const char* message)
{
    turn_off_all_valves();
    set_alarm(true);
    base.fsm.state = ERR;
    print_message(message);

    HAL_TIM_Base_Stop(&WATCHDOG_MS_TIMER);
    HAL_TIM_Base_Stop(&AUX_PHASE_MS_TIMER);
}

static void print_message(const char* message)
{
    CHECK(strlen( message) <= 20, "message string too long.");
    memset(base.aux_tab, ' ', AUX_TAB_LEN);
    snprintf((char*)base.aux_tab, AUX_TAB_LEN, "%-20s", message);
    Lcd_write((char*) base.aux_tab, MESSAGE_ROW, MESSAGE_COL);

    return;

    error:
    while(1);
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
