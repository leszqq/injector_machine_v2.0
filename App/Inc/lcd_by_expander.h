/*
 * lcd_by_expander.h
 *
 *  Created on: 8 mar 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file contain drivers for HD44780 LCD display through MCP23S08 GPIO expander in 4 bit mode.
 *      It is intended to use in real time applications along with HAL drivers.
 *
 *      HOW TO USE
 *      Initialize LCD with lcd_init() function.
 *
 *      Push commands to send to LCD in queue using API functions:
 *
 *      Call lcd_process() function, which will transmit data from queue using DMA.
 *      lcd_process() function should be placed inside main loop.
 */

#ifndef APP_INC_LCD_BY_EXPANDER_H_
#define APP_INC_LCD_BY_EXPANDER_H_

/* == exported includes == */
#include "main.h"
#include "gpio_expander.h"
/* == exported types == */

/* use this enum to fill resolve table based on connections between GPIO expander and LCD.
 * Then pass resolve table to init function. */
enum expander_pin {
    GP0 = 0x01,
    GP1 = 0x02,
    GP2 = 0x04,
    GP3 = 0x08,
    GP4 = 0x10,
    GP5 = 0x20,
    GP6 = 0x40,
    GP7 = 0x80
};

/* instantiate this struct, initialize it with enum expander_pin and pass to init_lcd() */
struct Resolve_table {
    enum expander_pin E, RS, RW, D7, D6, D5, D4;
};




/* == API functions == */
/**
 * @brief initialization function
 * @param hspi              -   pointer to SPI connected with MCP23S08 handle
 * @param device_address    -   address of the expander dev
 * @param CS_pin_port       -   expander CS pin port
 * @param CS_pin            -   expander CS pin
 * @param pin_resolve_table -   table containing pin resolve
 * @retval enum status
 */
enum GPIO_expander_status Lcd_init(SPI_HandleTypeDef* hspi, uint8_t device_address, GPIO_TypeDef *CS_pin_port, uint16_t CS_pin, struct Resolve_table resolve_tab);

/**
 * @brief push text to write on display at specified row and column in FIFO queue
 * @param text              -   text to write
 * @param row               -   row to write
 * @param col               -   collumn to write
 */
enum GPIO_expander_status Lcd_write(char *text, uint8_t row, uint8_t col);

/**
 * @brief push command to make LCD field blinking in FIFO queue
 * @param row               -   row of blinking field
 * @param col               -   col of blinking field
 * @retval enum status
 */
enum GPIO_expander_status Lcd_blink_on(uint8_t row, uint8_t col);

/**
 * @brief push command to turn off LCD field blinking in FIFO queue
 * @retval enum status
 */
enum GPIO_expander_status Lcd_blink_off();


/* execute tasks stored in FIFO queue - start DMA based communication with LCD */
enum GPIO_expander_status Lcd_process();

#endif /* APP_INC_LCD_BY_EXPANDER_H_ */
