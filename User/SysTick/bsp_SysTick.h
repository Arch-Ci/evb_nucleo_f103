#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

void bsp_timer_init(void);
void bsp_start_timer(int timerId, unsigned short millTime, void (*callback)(int id));
void bsp_stop_timer(int timerId);
void bsp_timer_proc(void);

void SysTick_Init(uint32_t ms);
void SysTick_Enable(unsigned char enable);
void HAL_IncTick(void);
unsigned long long HAL_GetTick(void);


#endif /* __SYSTICK_H */
