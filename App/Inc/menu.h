/*
 * menu.h
 *
 *  Created on: 23 mar 2021
 *      Author: Administrator
 */

#ifndef APP_INC_MENU_H_
#define APP_INC_MENU_H_

/* === exported includes === */
#include "gpio_expander.h"
#include "status_tab.h"
#include "encoder.h"

/* === exported defines === */
/* change these defines according to application */

#define DEBOUNCING_TIME         20
#define SPI_TIMEOUT
#define SEV_SEG_CS_PORT
#define SEV_SEG_CS_PIN
#define SEV
/* === API functions === */

/**
 * @brief menu initialization function. Initialize seven segment display, encoder and lcd display before
 *      menu initialization.
 * @param time_tab              - pointer to time table containing cycle defining periods
 * @param status_tab            - pointer to tab where error codes are preserved
 */
void menu_init(struct Time_table *time_tab, struct Status_tab *status_tab);

void menu_process();
#endif /* APP_INC_MENU_H_ */
