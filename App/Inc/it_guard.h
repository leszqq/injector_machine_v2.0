/*
 * it_guard.h
 *
 *  Created on: 16 mar 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file contain macros for disabling all IT requests during critical operations.
 *      use IT_GUARD_START before critical code, and IT_GUARD_END after it. IT_GUARD_END won't enable
 *      interrupts if they were disabled when IT_GUARD_START macro was used.
 */

#ifndef APP_INC_IT_GUARD_H_
#define APP_INC_IT_GUARD_H_

#define IT_GUARD_START              uint32_t prim = __get_PRIMASK(); __disable_irq();

#define IT_GUARD_END                if(!prim) __enable_irq();

#endif /* APP_INC_IT_GUARD_H_ */
