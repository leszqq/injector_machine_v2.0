/*
 * encoder.c
 *
 *  Created on: Mar 20, 2021
 *      Author: Wiktor Lechowicz
 */

/* === private includes === */
#include "stm32f0xx_hal.h"
/* === private defines === */
#define TIMER_POL_VAL       32768
#define TIMER_MAX_VAL       65535

/* === private variables */
static struct timer {
    TIM_HandleTypeDef   *htim;
} base;

/**
 * @brief initialize encoder
 */
void Encoder_init(TIM_HandleTypeDef* htim)
{
    base.htim = htim;

    base.htim->Instance->CNT = TIMER_POL_VAL;
    HAL_TIM_Encoder_Start(base.htim, TIM_CHANNEL_ALL);

}

/**
 * @brief get number of encoder transitions since last call of this function or init function
 * @retval  -   number of encoder transitions. It might be negative when turned counter-clockwise
 */
int16_t Encoder_get_transitions()
{
    int16_t res = READ_REG(base.htim->Instance->CNT);

    res -= TIMER_POL_VAL;
    if(res != 0) WRITE_REG(base.htim->Instance->CNT, TIMER_POL_VAL);

    return res;
}
