/*
 * eeprom_ic.c
 *
 *  Created on: Mar 19, 2021
 *      Author: Wiktor Lechowicz
 *
 *      This file contain code to read setups from EEPROM chip. Setups are 16 bit long, eeprom memory are 8 bit long. Setups are stored in little endian format.
 */

/* === private includes === */
#include "eeprom_ic.h"
#include "check_macros.h"
#include <assert.h>

/* === private defines === */
#define IC_ADDR                 0xA0
#define SETUP_ADDR              0x00

#define SETUP_MEM_SIZE          8

#define OPEN_TIME_INDEX         0
#define INJECTION_TIME_INDEX    2
#define COOLING_TIME_INDEX      4
#define MAX_CYCLE_CNT_INDEX     6

#define TIMEOUT                 1

/* === private variables === */
static struct EEPROM {
    I2C_HandleTypeDef*  hi2c;
} base;


/* === static functions === */
enum EEPROM_status resolve_status(HAL_StatusTypeDef status)
{
    if(status == HAL_OK){
        return EEPROM_OK;
    } else {
        return EEPROM_ERROR;
    }
}
/* === API functions === */

enum EEPROM_status EEPROM_init(I2C_HandleTypeDef* hi2c)
{
    base.hi2c = hi2c;

    return EEPROM_OK;
}

enum EEPROM_status EEPROM_read_setups(struct Time_table *time_tab, uint16_t *max_cnt_value)
{
    assert(time_tab != NULL);
    assert(max_cnt_value != NULL);

    HAL_StatusTypeDef stat;

    uint8_t temp_tab[SETUP_MEM_SIZE];
    stat = HAL_I2C_Mem_Read(base.hi2c, IC_ADDR, SETUP_ADDR, 1, temp_tab, SETUP_MEM_SIZE, TIMEOUT);
    CHECK(stat == HAL_OK);

    /* create uint16_t values based on MSB and LSB bytes read from memory */
    time_tab->open_time = (temp_tab[OPEN_TIME_INDEX] << 8) + temp_tab[OPEN_TIME_INDEX + 1];
    time_tab->injection_time = (temp_tab[INJECTION_TIME_INDEX] << 8) + temp_tab[INJECTION_TIME_INDEX + 1];
    time_tab->cooling_time = (temp_tab[COOLING_TIME_INDEX] << 8) + temp_tab[COOLING_TIME_INDEX + 1];

    *max_cnt_value = (temp_tab[MAX_CYCLE_CNT_INDEX] << 8) + temp_tab[MAX_CYCLE_CNT_INDEX + 1];

    return EEPROM_OK;

    error:
    return resolve_status(stat);

}

enum EEPROM_status EEPROM_write_setups(struct Time_table *time_tab, uint16_t max_cnt_value)
{
    /* parse MSB and LSB parts of data to write */
    uint8_t temp_tab[SETUP_MEM_SIZE];
    temp_tab[OPEN_TIME_INDEX] = time_tab->open_time >> 8;
    temp_tab[OPEN_TIME_INDEX + 1] = time_tab->open_time & 0xFF;

    temp_tab[INJECTION_TIME_INDEX] = time_tab->injection_time >> 8;
    temp_tab[INJECTION_TIME_INDEX + 1] = time_tab->injection_time & 0x00FF;

    temp_tab[COOLING_TIME_INDEX] = time_tab->cooling_time >> 8;
    temp_tab[COOLING_TIME_INDEX + 1] = time_tab->cooling_time & 0xFF;

    temp_tab[MAX_CYCLE_CNT_INDEX] = max_cnt_value >> 8;
    temp_tab[MAX_CYCLE_CNT_INDEX + 1] = max_cnt_value & 0xFF;

    /* write data to EEPROM */
    return(resolve_status(HAL_I2C_Mem_Write(base.hi2c, IC_ADDR, SETUP_ADDR, 1, temp_tab, SETUP_MEM_SIZE, TIMEOUT)));
}
