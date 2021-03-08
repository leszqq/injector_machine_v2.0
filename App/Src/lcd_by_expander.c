/*
 * lcd_by_expander.c
 *
 *  Created on: 8 mar 2021
 *      Author: Wiktor Lechowicz
 *      Description in header
 */

/* == private defines == */
#define ROW_0_ADDR 0x00
#define ROW_1_ADDR 0x40
#define ROW_2_ADDR 0x14
#define ROW_3_ADDR 0x54


/* == includes == */
#include "lcd_by_expander.h"
#include "spi.h"
#include "gpio_expander.h"


/* == private types == */

enum command {
    INIT_SEQ    = 0x03,

    CLEAR       = 0x01,

    TURN_OFF    = 0x08,

    /* Choose entry mode and shift mode, tehn write logic sum as 1 command */
    ENTRY_INC   = 0x06,
    ENTRY_DEC   = 0x04,
    SHIFT_ON    = 0x05,
    SHIFT_OFF   = 0x04,

    /* Do logic sum of what you want to turn on and write as one command */
    DISP_ON     = 0x0C,
    CURSOR_ON   = 0x0A,
    CHAR_BLINK  = 0x09,


    /* Choose data length, font, lines and do logic sum of them and send as 1 command */
    DATA_LEN_8B = 0x30,
    DATA_LEN_4B = 0x20,
    FONT_5x8    = 0x20,
    FONT_5x10   = 0x24,
    TWO_LINES   = 0x28,
    ONE_LINE    = 0x20,

    /* Do logic sum with address value and send as 1 command */
    SET_ADDR    = 0x80
};

/* == variables == */

static struct lcd {
    struct Resolve_table res_tab;                       // contains info about connections between expander and lcd

} base;


/* == static functions == */


enum message_type {
    COMMAND = 0,
    DATA = 1
};

enum write_mode {
    POLLING = 0,
    FIFO = 1
};
static enum GPIO_expander_status send_nibble(uint8_t nibble, enum message_type type, enum write_mode mode)
{
    uint8_t temp = 0;
    enum GPIO_expander_status status = GPIO_EXPANDER_OK;

    /* decode expected LCD inputs state and write them */
    if(type == COMMAND) temp |= base.res_tab.RS;

    if(nibble & 0x08) temp |= base.res_tab.D7;
    if(nibble & 0x04) temp |= base.res_tab.D6;
    if(nibble & 0x02) temp |= base.res_tab.D5;
    if(nibble & 0x01) temp |= base.res_tab.D4;

    /* write nibble in chosen mode */
    if(mode == POLLING){
        status = GPIO_expander_write(temp);
        if(status != GPIO_EXPANDER_OK) return status;

        status = GPIO_expander_write(temp | base.res_tab.E);
        if(status != GPIO_EXPANDER_OK) return status;

        status = GPIO_expander_write(temp);
        return status;

    } else if(mode == FIFO) {
        status = GPIO_expander_FIFO_write(temp);
        if(status != GPIO_EXPANDER_OK) return status;

        status = GPIO_expander_FIFO_write(temp | base.res_tab.E);
        if(status != GPIO_EXPANDER_OK) return status;

        status = GPIO_expander_FIFO_write(temp);
        return status;
    }
    return status;
}

/* push write sequence to expander */
static enum GPIO_expander_status send_command(enum command com, enum write_mode mode)
{
    enum GPIO_expander_status status = GPIO_EXPANDER_OK;

    status = send_nibble((uint8_t)(com) & 0xF0, COMMAND, mode);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_nibble((uint8_t)(com) & 0x0F, COMMAND, mode);
    return status;
}

static enum GPIO_expander_status write_byte(uint8_t byte, enum write_mode mode)
{
    enum GPIO_expander_status status = GPIO_EXPANDER_OK;

    status = send_nibble(byte & 0xF0, DATA, mode);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_nibble(byte & 0x0F, DATA, mode);
    return status;
}

/* API functions */
enum GPIO_expander_status lcd_init(SPI_HandleTypeDef* hspi, uint8_t device_address, GPIO_TypeDef *CS_pin_port, uint16_t CS_pin, struct Resolve_table resolve_tab)
{
    enum GPIO_expander_status status = GPIO_EXPANDER_OK;

    /* initialzie GPIO expander */
    GPIO_expander_init(hspi, device_address, CS_pin_port, CS_pin);

    /* store copy of resolution table */
    base.res_tab = resolve_tab;

    /* send initialization sequence in polling mode */
    HAL_Delay(50);
    send_nibble((uint8_t)(INIT_SEQ), COMMAND, POLLING);
    HAL_Delay(4);
    send_nibble((uint8_t)(INIT_SEQ), COMMAND, POLLING);
    HAL_Delay(1);
    send_nibble((uint8_t)(INIT_SEQ), COMMAND, POLLING);
    send_nibble(0x02, COMMAND, POLLING);

    status = send_command(DATA_LEN_4B | TWO_LINES | FONT_5x8, POLLING);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_command(TURN_OFF, POLLING);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_command(CLEAR, POLLING);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_command(ENTRY_INC | SHIFT_OFF, POLLING);
    if(status != GPIO_EXPANDER_OK) return status;

    status = send_command(DISP_ON, POLLING);
    return status;
}


enum GPIO_expander_status lcd_write(char *text, uint8_t row, uint8_t col)
{
    if(text == NULL) return GPIO_EXPANDER_ERROR;

    uint8_t i = 0;
    enum GPIO_expander_status status = GPIO_EXPANDER_OK;

    /* set row and column */
    uint8_t address = 0;
    switch(row){
    case 0:
        address = ROW_0_ADDR;
        break;
    case 1:
        address = ROW_1_ADDR;
        break;
    case 2:
        address = ROW_2_ADDR;
        break;
    case 3:
        address = ROW_3_ADDR;
        break;
    default:
        return GPIO_EXPANDER_ERROR;
    }

    address += (col % 20);
    status = send_command(SET_ADDR | address, FIFO);
    while(text[i] != '\0'){
        status = write_byte(text[i++], FIFO);
        if(status != GPIO_EXPANDER_OK) return status;
    }
    return GPIO_EXPANDER_OK;

}

/* execute tasks stored in FIFO queue - start DMA based communication with LCD */
enum GPIO_expander_status lcd_process()
{
    return GPIO_expander_process();
}

