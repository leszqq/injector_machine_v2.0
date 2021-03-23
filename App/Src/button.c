/*
 * button.c
 *
 *  Created on: 21 mar 2021
 *      Author: Administrator
 */

/* === private includes === */
#include "button.h"
#include "stm32f0xx_hal.h"


/* === API functions === */
void Button_init(struct Button *button, GPIO_TypeDef *port, uint16_t pin, uint8_t debouncing_time, GPIO_PinState active_level)
{
    button->port = port;
    button->pin = pin;

    button->debouncing_time = debouncing_time;
    button->active_level = active_level;

    button->been_push = false;
    button->old_state = false;
    button->new_state = false;
    button->time_stamp = HAL_GetTick();

}


bool Button_been_push(struct Button *button)
{
    if(button->been_push){
        button->been_push = false;
        return true;
    }
    return false;
}


void Button_process(struct Button *button)
{
    if( HAL_GetTick() > button->time_stamp + button->debouncing_time ){
        button->time_stamp = HAL_GetTick();

        button->new_state = (HAL_GPIO_ReadPin(button->port, button->pin) == button->active_level);

        if(button->new_state && (button->old_state == false)){
            button->been_push = true;
        }
        button->old_state = button->new_state;
    }
}
