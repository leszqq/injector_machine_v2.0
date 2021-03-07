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
 *      Set application specific fields by calling the GPIO_expander_init() function.
 *
 *      Set task using FIFO mode API functions and insert GPIO_expander_process()
 *      function inside main loop to attempt execute them using DMA once resources are available.
 *
 *      You can also use functions in polling mode to force assignments immediately.
 */

#ifndef APP_INC_GPIO_EXPANDER_H_
#define APP_INC_GPIO_EXPANDER_H_



/* == includes == */
#include "main.h"
#include "spi.h"


/* == exported types == */
enum GPIO_expander_status {
    GPIO_EXPANDER_OK = 0,
    GPIO_EXPANDER_BUSY,
    GPIO_EXPANDER_QUEUE_FULL,
    GPIO_EXPANDER_ERROR
};


/* == exported function prototypes == */

/**
 * @brief initialization function
 * @param hspi           -   pointer to SPI connected with MCP23S08 handle
 * @param device_address -   address of the expander dev
 * @param CS_pin_port    -   CS pin port
 * @param CS_pin         -   CS pin
 * @retval enum status
 */
void GPIO_expander_init(SPI_HandleTypeDef* hspi, uint8_t device_address, GPIO_TypeDef *CS_pin_port, uint16_t CS_pin);

/**
 * @brief this is the process executing tasks assigned with API functions below.
 *          Write this function inside main loop.
 * @retval status
 */
enum GPIO_expander_status GPIO_expander_process();


/* == API functions == */
/**
 * @brief Assign task in FIFO queue to write expander output pins GP7-GP0 on GPIO_expander_process() call,
 * @param byte - value to write to GP7-GP0 pins
 * @retval enum status
 */
enum GPIO_expander_status GPIO_expander_FIFO_write(uint8_t byte);

/**
 * @brief write expander output pins GP7-GP0 in polling mode.
 * @retval enum status
 * @param byte - value to write to GP7-GP0 pins
 * @param timeout - maximum time of expectancy for SPI peripheral availability
 * @retval enum status
 */
enum GPIO_expander_status GPIO_expander_write(uint8_t byte, uint16_t timeout);

#endif /* APP_INC_GPIO_EXPANDER_H_ */
