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
enum EEPROM_status EEPROM_init(I2C_HandleTypeDef* hi2c, uint32_t timeout);

/**
 * @brieg read up to 8 bytes from EEPROM IC in polling mode.
 * @param addr      -   memory address, it must be the multiply of 8 to assert that all 8 bytes will be read properly.
 * @param p_data    -   pointer to array where data will be stored.
 * @param size      -   number of bytes to read.  Must be less than 8.
 * @retval enum EEPROM_status
 */
enum EEPROM_status EEPROM_read(uint8_t addr, uint8_t *p_data, uint8_t size);

/**
 * @brief write up to 8 bytes to EEPROM IC in polling mode.
 * @param addr      -   memory address, it must be the multiply of 8 to assert that all 8 bytes will be written properly.
 * @param p_data    -   pointer to array where data to write is stored.
 * @param size      -   number of bytes form p_data array to write to EEPROM IC. Must be less than 8.
 * @retval enum EEPROM_status
 */
enum EEPROM_status EEPROM_write(uint8_t addr, uint8_t *p_data, uint8_t size);



#endif /* APP_INC_EEPROM_IC_H_ */
