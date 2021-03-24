/*
 * sev_seg_disp.c
 *
 *  Created on: 15 mar 2021
 *      Author: Administrator
 */

/* === private defines === */
#define AUX_TAB_SIZE    10      // size of auxiliary table for data operations
#define BUFF_SIZE       50
#define SEG_BLANK       0x0F    // codeto write into MAX_7219_DIGIT_x register for blank digit, used for blinking
#define MAX_NUMBER      9999    // biggest number which can be displayed
#define BLINK_PERIOD    100U    // blink period in ms
/* === private macros */





/* === private includes === */
#include "stm32f0xx_hal.h"
#include "sev_seg_disp.h"
#include <stdbool.h>
#include "spi.h"
#include <string.h>
#include "it_guard.h"
#include "check_macros.h"

/* private types */

/* register to write to by write_register() function */
enum registers {
    SHUTDOWN    =   0x0C,
    DEC_MODE    =   0x09,
    INTENSITY   =   0x0A,
    SCAN_LIM    =   0x0B,
    DISP_TEST   =   0x0F,

    DIGIT_0     =   0x01,
    DIGIT_1     =   0x02,
    DIGIT_2     =   0x03,
    DIGIT_3     =   0x04
};

/* === private types === */

/* Pair of register and byte which will should be written to this register */
struct reg_byte_pair {
    enum registers      reg;
    uint8_t             byte;
};

/* FIFO queue of register-byte pairs to send */
struct Write_buffer{
    uint8_t                     amount_to_send;
    struct reg_byte_pair        r_b_pairs[BUFF_SIZE];
};

/* === private variables === */
static struct Display {
    uint16_t            number;                         // displayed number

    enum sev_seg_digit  blinking_digit;                 // blinking digit
    bool                blink_flag;                     // flag for blinking purposes, true if digit is off
    uint32_t            timestamp;                      // time of last blink_flag toggle

    SPI_HandleTypeDef*  hspi;                           // SPI handle pointer
    GPIO_TypeDef*       CS_port;                        // MAX7219 CS pin port
    uint16_t            CS_pin;                         // MAX7219 CS pin

    uint8_t             aux_tab[AUX_TAB_SIZE];          // auxiliary tab

    struct Write_buffer buff;                           // buffer with register-byte pairs to write

    uint32_t            timeout;                        // timeout duration for polling mode transmissions
    enum sev_seg_status status;                         // error code of last operation. Despite the fact, that all the functions return status, it is
                                                        // preserved to make code more concise with CHECK macro and "error" label.
} base;

/* === static functions === */

/* resolve HAL error codes to sev_seg_status codes */
static enum sev_seg_status resolve_status(HAL_StatusTypeDef hal_stat){
    switch(hal_stat){
    case HAL_OK:
        return SEV_SEG_OK;
        break;
    case HAL_ERROR:
    default:
        return SEV_SEG_ERROR;
        break;
    case HAL_BUSY:
        return SEV_SEG_BUSY;
        break;
    case HAL_TIMEOUT:
        return SEV_SEG_TIMEOUT;
        break;
    }
}

/* write MAX7219 register through SPI in polling mode */
static enum sev_seg_status write_reg(enum registers reg, uint8_t byte){
    /* prepare data to send */
    base.aux_tab[0] = reg;
    base.aux_tab[1] = byte;


    if(HAL_SPI_GetState(base.hspi) != HAL_SPI_STATE_READY) return SEV_SEG_BUSY;

    IT_GUARD_START

    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_RESET);
    base.status = resolve_status(HAL_SPI_Transmit(base.hspi, base.aux_tab, 2, base.timeout));
    CHECK(base.status == SEV_SEG_OK);
    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_SET);

    IT_GUARD_END
    return SEV_SEG_OK;


error:
    IT_GUARD_END
    HAL_GPIO_WritePin(base.CS_port, base.CS_pin, GPIO_PIN_SET);
    return base.status;
}

/* put next register - byte pair to send in process() function */
static enum sev_seg_status put_in_buff(enum registers reg, uint8_t byte){

    if(base.buff.amount_to_send == BUFF_SIZE){
        base.status = SEV_SEG_FULL;
        return base.status;
    }

    base.buff.r_b_pairs[base.buff.amount_to_send].reg = reg;
    base.buff.r_b_pairs[base.buff.amount_to_send++].byte = byte;

    return SEV_SEG_OK;
}

/* put command in buffer to write given digit or blank segment on digit_position */
static enum sev_seg_status write_digit(enum sev_seg_digit digit_pos, uint8_t digit, bool blank){

    if(blank){
        CHECK(put_in_buff( (enum registers) digit_pos, SEG_BLANK) == SEV_SEG_OK);
    } else {
        CHECK(put_in_buff( (enum registers) digit_pos, digit) == SEV_SEG_OK);
    }

    return SEV_SEG_OK;

    /* jump here if any CHECK condition not met */
error:
    return base.status;
}

/* return THOUSANDS, HUDNREDS, TENS or UNITS digit of number */
uint8_t get_digit(uint16_t number, enum sev_seg_digit digit){
    switch(digit){
    case THOUSANDS:
        return number / 1000;
    case HUNDREDS:
        return (number % 1000) / 100;
    case TENS:
        return (number % 100) / 10;
    case UNITS:
    default:
        return number % 10;
    }
}
/* === API function === */

/* initialization function */
enum sev_seg_status Sev_seg_init(SPI_HandleTypeDef* hspi, uint32_t timeout, GPIO_TypeDef* CS_port, uint16_t CS_pin){

    /* initialize base struct */
    base.number = 0;
    base.blinking_digit = NONE;
    base.blink_flag = true;
    base.hspi = hspi;

    base.CS_port = CS_port;
    base.CS_pin = CS_pin;
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);


    memset(base.aux_tab, 0, AUX_TAB_SIZE);
    base.buff.amount_to_send = 0;
    base.timeout = timeout;

    base.status = SEV_SEG_OK;

    /* write configuration sequence to MAX7219 IC */
    CHECK(write_reg(SHUTDOWN, 0x01) == SEV_SEG_OK);
    CHECK(write_reg(DEC_MODE, 0xFF) == SEV_SEG_OK);              // decoding mode on
    CHECK(write_reg(INTENSITY, 0x02) == SEV_SEG_OK);
    CHECK(write_reg(SCAN_LIM, 0x03) == SEV_SEG_OK);
    CHECK(write_reg(DISP_TEST, 0x00) == SEV_SEG_OK);

    /* set display blank */
    CHECK(write_reg(DIGIT_3, SEG_BLANK) == SEV_SEG_OK);
    CHECK(write_reg(DIGIT_2, SEG_BLANK) == SEV_SEG_OK);
    CHECK(write_reg(DIGIT_1, SEG_BLANK) == SEV_SEG_OK);
    CHECK(write_reg(DIGIT_0, SEG_BLANK) == SEV_SEG_OK);

    return SEV_SEG_OK;

/* jump here if any CHECK condition is not true */
error:
    return base.status;
}

/**
 * @brief write number to seven segment display
 * @param number - number to display
 * @retval status
 */
enum sev_seg_status Sev_seg_write(uint16_t number){

    if(number > 9999){
        base.status = SEV_SEG_ERROR;
        return base.status;
    }

    /* thousands digit */
    uint8_t temp = get_digit(number, THOUSANDS);

    bool blank_flag = true;                                             // this flag is true until any digit is greater than 0.
                                                                        // This is used to not display leading zeros.
    if(blank_flag && (temp != 0) ) blank_flag = false;

    /* check if new thousands digit is different from already displayed */
    if(temp != get_digit(base.number, THOUSANDS) ){
        CHECK( write_digit(DIGIT_3, temp, blank_flag) == SEV_SEG_OK );
    }

    /* hundreds digit */
    temp = get_digit(number, HUNDREDS);
    if(blank_flag && (temp != 0) ) blank_flag = false;

    /* check if new hundreds digit is different from already displayed */
    if(temp != get_digit(base.number, HUNDREDS) ){
        CHECK( write_digit(DIGIT_2, temp, blank_flag) == SEV_SEG_OK );
    }

    temp = get_digit(number, TENS);
    if(blank_flag && (temp != 0) ) blank_flag = false;

    /* check if new tens digit is different from already displayed */
    if(temp != get_digit(base.number, TENS) ){
        CHECK( write_digit(DIGIT_1, temp, blank_flag) == SEV_SEG_OK );
    }

    temp = get_digit(number, UNITS);
    blank_flag = false;                                                     // always display unit digit

    /* check if new unit digit is different from already displayed */
    if(temp != get_digit(base.number, UNITS) ){
        CHECK( write_digit(DIGIT_0, temp, blank_flag) == SEV_SEG_OK );
    }

    /* update currently displayed number copy in base struct */
    base.number = number;

    return SEV_SEG_OK;

    /* jump here if any CHECK condition not met */
error:
    return base.status;
}


enum sev_seg_status Sev_seg_blink(enum sev_seg_digit digit){

    if((base.blink_flag = true) ){
        /* Turn on digit which have been blinking before */
        CHECK( write_digit(base.blinking_digit, get_digit(base.number, base.blinking_digit), false) == SEV_SEG_OK);
    }
    base.blinking_digit = digit;
    base.blink_flag = true;


    return SEV_SEG_OK;

error:
    return base.status;
}


enum sev_seg_status Sev_seg_process(){

    /* blinking */
    if(base.blinking_digit != NONE){
        if(HAL_GetTick() > base.timestamp + BLINK_PERIOD){
            base.timestamp = HAL_GetTick();

            if(base.blink_flag){
                /* set segment blank */
                CHECK(write_digit(base.blinking_digit, 0, true) == SEV_SEG_OK);         // digit value set to 0 because it is blank anyway
            } else {
                /* display digit */
                CHECK( write_digit(base.blinking_digit, get_digit(base.number, base.blinking_digit), false ) == SEV_SEG_OK);
            }

            base.blink_flag = !base.blink_flag;
            base.timestamp = HAL_GetTick();
        }
    }

    /* send next data from buffer */
    if(base.buff.amount_to_send != 0){
        CHECK(write_reg(base.buff.r_b_pairs[base.buff.amount_to_send-1].reg,
                        base.buff.r_b_pairs[base.buff.amount_to_send-1].byte) == SEV_SEG_OK);

        base.buff.amount_to_send--;
        if(base.status == SEV_SEG_FULL) base.status = SEV_SEG_OK;
    }

    return SEV_SEG_OK;

error:
    return base.status;
}


// TODO: add IT guard in polling writes, inti function, write function
