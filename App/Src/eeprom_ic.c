/*
 * eeprom_ic.c
 *
 *  Created on: Mar 19, 2021
 *      Author: Administrator
 */

/* === private includes === */
#include "eeprom_ic.h"


/* === private defines === */
#define IC_ADDR         0xA0


/* === private variables === */
static struct EEPROM {
    I2C_HandleTypeDef*  hi2c;
    uint32_t            timeout;
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

enum EEPROM_status EEPROM_init(I2C_HandleTypeDef* hi2c, uint32_t timeout)
{
    base.hi2c = hi2c;
    base.timeout = timeout;

    return EEPROM_OK;
}

enum EEPROM_status EEPROM_read(uint8_t addr, uint8_t *p_data, uint8_t size)
{
    // TODO
    return resolve_status(HAL_I2C_Mem_Read(base.hi2c, IC_ADDR, addr, 1, p_data, size, base.timeout));
}

enum EEPROM_status EEPROM_write(uint8_t addr, uint8_t *p_data, uint8_t size)
{
    // TODO
    return resolve_status(HAL_I2C_Mem_Write(base.hi2c, IC_ADDR, addr, 1, p_data, size, base.timeout));
}
