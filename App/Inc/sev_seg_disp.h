/*
 * sev_seg_disp.h
 *
 *  Created on: 15 mar 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file containt code for displaying four digits on 7-segment display via MAX7219 IC.
 *      Blinking is implemented as well.
 *
 *      Push commands sequence to send using sev_seg_write() and sev_seg_blink() functions.
 *      There is no difference of what is the precedence of commands, so they are stored in stack
 *      and pop from it one by one by sev_seg_process() function.
 *
 *      sev_seg_init() initializes IC and display in pooling mode
 *
 *      put sev_seg_process() function inside main loop to execute task set with another functions.
 *      sev_seg_process() is writing one byte to one register at call.
 */


#ifndef APP_INC_SEV_SEG_DISP_H_
#define APP_INC_SEV_SEG_DISP_H_


/* display status */
enum sev_seg_status {
    SEV_SEG_OK,
    SEV_SEG_BUSY,
    SEV_SEG_FULL,
    SEV_SEG_ERROR,
    SEV_SEG_TIMEOUT
};


/**
 * @brief Initialization function. After initialization, there is "0" on display.
 * @param hspi      -   pointer to SPI handle
 * @param timeout   -   timeout duration for polling transmissions
 * @param CS_port   -   MCU port to which CS pin of MAX7219 is connected
 * @param CS_pin    -   MCU pin to which CS pin of MAX7219 is connected
 * @retval status
 */
enum sev_seg_status sev_seg_init(SPI_HandleTypeDef* hspi, uint32_t timeout, GPIO_TypeDef* CS_port, uint16_t CS_pin);


/**
 * @brief write number to seven segment display
 * @param number - number to display
 * @retval status
 */
enum sev_seg_status sev_seg_write(uint16_t number);


/* enum type for selecting blinking digit */
enum sev_seg_digit {
    NONE        =   0x00,
    UNITS       =   0x01,
    TENS        =   0x02,
    HUNDREDS    =   0x03,
    THOUSANDS   =   0x04
};
/**
 * @brief blink one of digits or stop blinking any
 * @param digit - NONE, UNITS, TENS, HUNDREDS or THOUSANDS
 * @retval status
 */
enum sev_seg_status sev_seg_blink(enum sev_seg_digit digit);


/**
 * @brief put this in main loop for blinking timing
 * @retval status
 */
enum sev_seg_status sev_seg_process();


// TODO: analyze IT cases and if IT there should be IT gaurds macros inside static polling write fuction. ???
#endif /* APP_INC_SEV_SEG_DISP_H_ */
