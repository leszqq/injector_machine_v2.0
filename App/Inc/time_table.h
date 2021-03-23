/*
 * time_table.h
 *
 *  Created on: 23 mar 2021
 *      Author: Wiktor Lechowicz
 *
 *
 *      This header defines type used by menu struct and fsm struct
 */

#ifndef APP_INC_TIME_TABLE_H_
#define APP_INC_TIME_TABLE_H_

/* === private includes === */
#include <stdint.h>

/* periods of time dependent machine cycle steps or time regimes to perform given operations */
struct Time_table {
    uint16_t open_time;
    uint16_t injection_time;
    uint16_t cooling_time;
};

#endif /* APP_INC_TIME_TABLE_H_ */
