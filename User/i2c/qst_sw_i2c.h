#ifndef _BSP_SW_I2C_H
#define _BSP_SW_I2C_H

#include <inttypes.h>
#include "stm32f10x.h"


void i2c_sw_gpio_config(void);
uint8_t qst_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat);
uint8_t qst_sw_writeregs(uint8_t slave, uint8_t reg_add, uint8_t *reg_dat, uint8_t len);
uint8_t qst_sw_readreg(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num);
uint8_t qmt200_sw_readreg(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num);

#endif
