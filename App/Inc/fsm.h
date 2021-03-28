/*
 * fsm.h
 *
 *  Created on: 23 mar 2021
 *      Author: Wiktor Lechowicz
 */

#ifndef APP_INC_FSM_H_
#define APP_INC_FSM_H_

/* === exported types === */

enum fsm_state {
    STOP = 0,
    PAUSE,
    FORM_CLOSING,
    INJECTION,
    COOLING,
    FORM_OPENING,
    ERR
};

/* state machine for maintaining machine cycle */
struct Fsm {
    enum fsm_state         state;
};

#endif /* APP_INC_FSM_H_ */
