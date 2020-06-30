#ifndef __PX318J_H
#define __PX318J_H
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "qst_sw_i2c.h"


#define PX318J_LOG		printf
#define PX318J_ERR		printf

#define PX318J_II_ADDR		(0x1c<<1)


uint8_t px318j_init(void);
uint8_t px318j_set_threshold(uint16_t high, uint16_t low);
uint16_t px318j_read_proximity(void);
uint8_t px318j_get_int_flag(void);

extern void qst_delay(unsigned int delay);

#endif

