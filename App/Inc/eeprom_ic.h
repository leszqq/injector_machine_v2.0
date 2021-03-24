/*
 * eeprom_ic.h
 *
 *  Created on: Mar 19, 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file contains code for storing and reading data from 24AA01-I/SN EEPROM chip
 *
 *      Read and write in polling mode.
 */

#ifndef APP_INC_EEPROM_IC_H_
#define APP_INC_EEPROM_IC_H_

#define IC_ADDR         0xA0

/* === exported includes === */
#include "i2c.h"
#include "main.h"
#include "stm32f0xx_hal_i2c.h"
#include "time_table.h"


/* === exported types === */
enum EEPROM_status {
    EEPROM_OK,              // data read or written succesfuly
    EEPROM_ERROR            // error occured during read or write
};

/* === API function === */

/**
 * @brief intialize EEPROM IC
 * @param *hi2c     -   pointer to i2c handle, to which IC is connected.
 * @retval enum EEPROM_status
 */
enum EEPROM_status EEPROM_init(I2C_HandleTypeDef* hi2c);

/**
 * @brief read saved setups from eeprom memory
 * @param time_tab          - pointer to table in which cycle times will be stored
 * @param max_cnt_cycles    - pointer to variable where maximum cycle count will be stored
 * @retval enum EEPROM_status
 */
enum EEPROM_status EEPROM_read_setups(struct Time_table *time_tab, uint16_t *max_cnt_value);

/**
 * @brief write setups to eeprom memory
 * @param time_tab          - pointer to table in which cycle times are stored.
 * @param max_cnt_value     - maximum value of cycle counter
 * @retval enum EEPROM status
 */
enum EEPROM_status EEPROM_write_setups(struct Time_table *time_tab, uint16_t max_cnt_value);



#endif /* APP_INC_EEPROM_IC_H_ */
