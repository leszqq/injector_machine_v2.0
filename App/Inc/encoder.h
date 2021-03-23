/*
 * encoder.h
 *
 *  Created on: 20 mar 2021
 *      Author: Wiktor Lechowicz
 *
 *      Code for reading increment encoder.
 *
 *      Init it with encoder_init() function.
 *
 *      Use read_encoder() function to get number of steps done since last call of encoder_get_transitions function.
 */

#ifndef APP_INC_ENCODER_H_
#define APP_INC_ENCODER_H_

/**
 * @brief initialize encoder
 * @param htim  -   pointer to timer configured in encoder mode handle.
 */
void Encoder_init(TIM_HandleTypeDef* tim);

/**
 * @brief get number of encoder transitions since last call of this function or init function
 * @retval  -   number of encoder transitions. It might be negative when turned counter-clockwise
 */
int16_t Encoder_get_transitions();


#endif /* APP_INC_ENCODER_H_ */
