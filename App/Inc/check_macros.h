/*
 * check_macros.h
 *
 *  Created on: 21 mar 2021
 *      Author: Administrator
 */

#ifndef APP_INC_CHECK_MACROS_H_
#define APP_INC_CHECK_MACROS_H_

/* if condition not met, jum to error label */
#define CHECK(S) if(!(S)) goto error

#endif /* APP_INC_CHECK_MACROS_H_ */
