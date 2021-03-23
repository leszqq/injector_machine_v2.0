/*
 * status_tab.h
 *
 *  Created on: 23 mar 2021
 *      Author: Wiktor Lechowicz
 */

#ifndef APP_INC_STATUS_TAB_H_
#define APP_INC_STATUS_TAB_H_

#include "gpio_expander.h"
#include "sev_seg_disp.h"

/* table for preserving error codes */
struct Status_tab {
    enum GPIO_expander_status   GPIO_exp_stat;
    enum sev_seg_status         sev_seg_stat;
};

#endif /* APP_INC_STATUS_TAB_H_ */
