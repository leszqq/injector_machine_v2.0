/*
 * button.c
 *
 *  Created on: 21 mar 2021
 *      Author: Administrator
 */

/* === private includes === */
#include <buttons.h>
#include "stm32f0xx_hal.h"
#include <assert.h>

/* === private defines === */
#define NUM_BUTTONS             3

/* === private_types === */
struct Button {
    GPIO_TypeDef               *port;                       // button pot
    uint16_t                    pin;                        // button pin
    uint32_t                    time_stamp;                 // time of last transition
    uint8_t                     debouncing_time;            // debouncing period
    GPIO_PinState               active_level;               // active logic level of button
    bool                        old_state, new_state;       // active - true, inactive - false
    bool                        been_push;                  // true if have been push recently
};

static struct Buttons {
    struct Button   buttons_tab[NUM_BUTTONS];
    uint8_t         next_free_id;
}base;

/* === API functions === */
button_id Button_init(GPIO_TypeDef *port, uint16_t pin, uint8_t debouncing_time, GPIO_PinState active_level)
{
    assert(base.next_free_id < NUM_BUTTONS);

    /* create new Button in button_table and associate it with id */
    button_id id = base.next_free_id;
    base.next_free_id++;

    struct Button *button = &base.buttons_tab[id];

    button->port = port;
    button->pin = pin;

    button->debouncing_time = debouncing_time;
    button->active_level = active_level;

    button->been_push = false;
    button->old_state = false;
    button->new_state = false;
    button->time_stamp = HAL_GetTick();

    return id;
}


bool Button_been_push(button_id id)
{
    assert(id < base.next_free_id);

    /* read and clear been_push flag.*/
    struct Button *button = &base.buttons_tab[id];
    if(button->been_push){
        button->been_push = false;
        return true;
    }
    return false;
}


void Buttons_process()
{
    struct Button *button;

    for(uint8_t i = 0; i < base.next_free_id; i++){
         button = &base.buttons_tab[i];

        /* check if button state changed from inactive to active. Store this information in been_push field. */
        if( HAL_GetTick() > button->time_stamp + button->debouncing_time ){
            button->time_stamp = HAL_GetTick();

            button->new_state = (HAL_GPIO_ReadPin(button->port, button->pin) == button->active_level);

            if(button->new_state && (button->old_state == false)){
                button->been_push = true;
            }
            button->old_state = button->new_state;
        }
    }
}
