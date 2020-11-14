#ifndef __QMT200_H
#define __QMT200_H
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "qst_sw_i2c.h"

#define QMT200_IIC_ADDR				0x00

int qmt200_init(void);
void qmt200_read_raw(void);

#endif

