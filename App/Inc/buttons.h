/*
 * button.h
 *
 *  Created on: 21 mar 2021
 *      Author: Administrator
 */

#ifndef APP_INC_BUTTON_H_
#define APP_INC_BUTTON_H_

/* === exported includes === */
#include <stdbool.h>
#include "stm32f0xx_hal.h"

/* === exported types === */

/* button handle for use with Button_.. functions */
typedef uint8_t                         button_id;

/* Do not modify or initialize fields of this structure. Instantiate it and use only as Button_ function arguments*/



/* === API functions === */

/**
 * @brief initialize new button and get it id.
 * @param button                -       pointer to button struct which will be used to specify button for Button_is_push() function
 * @param port                  -       button port
 * @param pin                   -       button pin
 * @param deboucing_time        -       debouncing time in ms
 * @param active_level          -       ACTIVE_HIGH or ACTIVE_LOW
 * @retval button id
 */
button_id Button_init( GPIO_TypeDef *port, uint16_t pin, uint8_t debouncing_time, GPIO_PinState active_level);


/**
 * @brief return true if button have been push since last call of this function
 * @param button    -   pointer to button structure
 * @retval true or false
 */
bool Button_been_push(button_id id);


/**
 * @brief put this in main loop.
 */
void Buttons_process();

#endif /* APP_INC_BUTTON_H_ */
