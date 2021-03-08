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
#define WRITE_BUFFER_SIZE                       1440                                // adjust this if you like
#define AUX_TAB_SIZE                               3


/* == private types == */

/* internal register addresses of MCP23S08 IC. Error-prone alternative for #defines */
enum IC_register {
    IODIR = 0x00,
    IOCON = 0x05,
    OLAT = 0x0A,
    GPIO = 0x09
};

struct Write_buffer {
    uint8_t                         amount_to_send;                                 // amount of writes ( register byte + data byte pairs)
    uint8_t                         queue[WRITE_BUFFER_SIZE];                       // bytes and their destination expander registers pairs
};

struct Aux_tab {
    uint8_t                         size;
    uint8_t                         data[AUX_TAB_SIZE];
};


/* == variables == */
static struct GPIO_expander {
    SPI_HandleTypeDef*              hspi;
    GPIO_TypeDef*                   CS_port;
    uint16_t                        CS_pin;
    uint8_t                         device_address;
    struct Write_buffer             write_buffer;
    struct Aux_tab                  aux_tab;
} base;

/* == static functions == */

/* write register in polling mode */
HAL_StatusTypeDef write_reg(enum IC_register reg, uint8_t data){
    /* prepare bytes sequence */
    base.aux_tab.data[0] = base.device_address;
    base.aux_tab.data[1] = (uint8_t) reg;
    base.aux_tab.data[2] = data;

    /* try transmitting it via SPI*/
    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef err_code = HAL_SPI_Transmit(base.hspi, base.aux_tab.data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_SET);

    return err_code;
}

enum GPIO_expander_status resolve_status(HAL_StatusTypeDef status)
{
    switch(status){
    case HAL_OK:
        return GPIO_EXPANDER_OK;
        break;
    case HAL_BUSY:
        return GPIO_EXPANDER_BUSY;
        break;
    case HAL_TIMEOUT:
    case HAL_ERROR:
    default:
        return GPIO_EXPANDER_ERROR;
        break;
    }
}

/* API functions == */

void GPIO_expander_init(SPI_HandleTypeDef* hspi, uint8_t device_address, GPIO_TypeDef *CS_pin_port, uint16_t CS_pin)
{
    /* base struct initialization */
    base.hspi = hspi;
    base.CS_port = CS_pin_port;
    base.CS_pin = CS_pin;
    base.device_address = device_address;

    /* initialize empty write buffer */
    struct Write_buffer buff = {
            .amount_to_send = 2,
            .queue = {device_address, OLAT, 0}
    };
    base.write_buffer = buff;

    /* initialize auxiliary tab with zeros */
    struct Aux_tab aux_tab = {
            .size = AUX_TAB_SIZE,
            .data = {0}                                             // this fills entire data array with zeros
    };
    base.aux_tab = aux_tab;

    /* IC initialization */
    write_reg(IODIR, 0x00);                                         // all outputs
    write_reg(OLAT, 0x00);                                          // all GPIOs low
    write_reg(IOCON, 0x20);                                         // disable sequential operation
}


enum GPIO_expander_status GPIO_expander_FIFO_write(uint8_t byte)
{
    /* assert that buffer is not full */
    struct Write_buffer *buff = &base.write_buffer;
    int next_index = buff->amount_to_send;
    if(next_index >= WRITE_BUFFER_SIZE - 1) return GPIO_EXPANDER_QUEUE_FULL;

    /* push byte to write on queue */
    buff->queue[next_index++] = byte;
    buff->amount_to_send = next_index;
    return GPIO_EXPANDER_OK;
}


enum GPIO_expander_status GPIO_expander_write(uint8_t byte)
{
    /* write MCP OLAT register */
    HAL_StatusTypeDef status = write_reg(OLAT, byte);

    /* return status depending on SPI transmission execution */
    return resolve_status(status);
}


enum GPIO_expander_status GPIO_expander_process(){
    /* start DMA transmission if write buffer is not empty and SPI peripheral is not busy */
    struct Write_buffer *buff = &base.write_buffer;
    if(buff->amount_to_send > 2){
        if(base.hspi->State == HAL_SPI_STATE_BUSY) return GPIO_EXPANDER_BUSY;

        HAL_StatusTypeDef status = HAL_OK;
        HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit_DMA(base.hspi, buff->queue, buff->amount_to_send);

        if(status == HAL_OK) buff->amount_to_send = 2;                                  // firt 2 bytes of queue are device and register addresses
        return resolve_status(status);
    } else {
        return GPIO_EXPANDER_OK;
    }
    return GPIO_EXPANDER_ERROR;
}

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi){
    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_SET);
}

