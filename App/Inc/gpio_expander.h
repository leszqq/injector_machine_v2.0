/*
 * gpio_expander.h
 *
 *  Created on: Feb 26, 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file contains code to interface with MCP23S08 GPIO expander.
 *      It is intended to use along with HAL drivers.
 *
 *      HOW TO USE
 *      To initialize expander, create struct GPIO_exp_init_t and assign
 *      suitable fields. Then, call the GPIO_expander_init() function.
 *
 *      Insert GPIO_expander_process() function inside main loop.
 *      Assign tasks to be done at GPIO_expander_process() call with API functions.
 *
 */

#ifndef APP_INC_GPIO_EXPANDER_H_
#define APP_INC_GPIO_EXPANDER_H_


/* == includes == */
#include "main.h"
#include "spi.h"


/* == exported types == */
enum GPIO_expander_status {
    GPIO_EXPANDER_OK = 0,
    GPIO_EXPANDER_ERROR
};

enum GPIO_expander_mode {
    GPIO_EXPANDER_MODE_BLOCKING = 0,
    GPIO_EXPANDER_MODE_FIFO
};


/* == exported function prototypes == */

/**
 * @brief initialization function
 * @param hspi          -   pointer to SPI connected with MCP23S08 handle
 * @param CS_pin_port   -   CS pin port
 * @param CS_pin        -   CS pin
 * @retval enum status
 */
void GPIO_expander_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef CS_pin_port, uint16_t CS_pin);

/**
 * @brief this is the process executing tasks assigned with API functions below.
 *          Write this function inside main loop.
 * @retval status
 */
enum GPIO_expander_status GPIO_expander_process();


/* == API functions == */
/**
 * @brief write expander output pins GP7-GP0
 * @param byte - value to write to GP7-GP0 pins
 * @param mode - GPIO_EXPANDER_MODE_BLOCKING:
 *                  wait until SPI peripheral is not busy and write expander outputs.
 *               GPIO_EXPANDER_MODE_FIFO:
 *                  try to send data if SPI is not busy, else store the task to FIFO
 *                  and make an attempt to execute at GPIO_expander_process() call.
 * @retval enum status
 */
enum GPIO_expander_status GPIO_expander_write_outputs(uint8_t byte, enum GPIO_expander_mode mode);

#endif /* APP_INC_GPIO_EXPANDER_H_ */
