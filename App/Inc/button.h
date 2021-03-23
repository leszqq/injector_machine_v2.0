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



/* Do not modify or initialize fields of this structure. Instantiate it and use only as Button_ function arguments*/
struct Button {
    GPIO_TypeDef               *port;                       // button pot
    uint16_t                    pin;                        // button pin
    uint32_t                    time_stamp;                 // time of last transition
    uint8_t                     debouncing_time;            // debouncing period
    GPIO_PinState               active_level;               // active logic level of button
    bool                        old_state, new_state;       // active - true, inactive - false
    bool                        been_push;                  // true if have been push recently
};


/* === API functions === */

/**
 * @param button                -       pointer to button struct which will be used to specify button for Button_is_push() function
 * @param port                  -       button port
 * @param pin                   -       button pin
 * @param deboucing_time        -       debouncing time in ms
 * @param active_level          -       ACTIVE_HIGH or ACTIVE_LOW
 */
void Button_init(struct Button *button, GPIO_TypeDef *port, uint16_t pin, uint8_t debouncing_time, GPIO_PinState active_level);


/**
 * @brief return true if button have been push since last call of this function
 * @param button    -   pointer to button structure
 * @retval true or false
 */
bool Button_been_push(struct Button *button);


/**
 * @brief put this in main loop
 */
void Button_process(struct Button *button);

#endif /* APP_INC_BUTTON_H_ */
