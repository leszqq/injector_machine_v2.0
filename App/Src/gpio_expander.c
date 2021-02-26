/*
 * gpio_expander.c
 *
 *  Created on: Feb 26, 2021
 *      Author: Wiktor Lechowicz
 */


/* == includes == */
#include "gpio_expander.h"
#include <stdbool.h>

/* == private defines == */
#define FIFO_LEN     5

/* == types == */
struct write_task {
    bool         pending;
    uint8_t         byte;
};

/* == variables == */
static struct GPIO_expander {
    SPI_HandleTypeDef*              handle;
    GPIO_TypeDef                    CS_port;
    uint16_t                        CS_pin;
    struct write_task               tx_fifo[FIFO_LEN];
    uint8_t                         fifo_index;
} base;

/* == static functions == */


/* API functions == */
void GPIO_expander_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef CS_pin_port, uint16_t CS_pin){
    base.handle = hspi;
    base.CS_port = CS_pin_port;
    base.CS_pin = CS_pin;
    for(uint8_t i = 0; i < FIFO_LEN; i++){
        base.tx_fifo[i].pending = false;
        base.tx_fifo[i].byte = 0;
    }
}

