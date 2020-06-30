/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
  *          常用的有 1us 10us 1ms 中断。     
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
//SysTick_Config主要用来配置中端向量，重置STK_VAL寄存器，配置SysTick时钟为AHB 
#include "./systick/bsp_SysTick.h"
static __IO unsigned long long uwTick = 0;
__IO unsigned int TickDelay = 0;
static unsigned int ms_per_tick = 0;

enum
{
	BSP_TIMER_0,
	BSP_TIMER_1,
	//BSP_TIMER_2,
	//BSP_TIMER_3,

	BSP_TIMER_MAX
};

typedef struct
{
	//unsigned short millTime;
	unsigned char  used;
	unsigned long long tick_satrt;
	unsigned short duration;
	void (*callback)(int timeId);
} bsp_timer_t;

static bsp_timer_t	bsp_timer_array[BSP_TIMER_MAX];

void bsp_timer_init(void)
{
	int i;

	for(i=0; i<BSP_TIMER_MAX; i++)
	{
		bsp_timer_array[i].used = 0;
		bsp_timer_array[i].tick_satrt = 0;
		bsp_timer_array[i].duration = 0;
		bsp_timer_array[i].callback = 0;
	}
}

void bsp_start_timer(int timerId, unsigned short millTime, void (*callback)(int id))
{
	if(timerId >= BSP_TIMER_MAX)
		return;

	bsp_timer_array[timerId].tick_satrt = HAL_GetTick();
	bsp_timer_array[timerId].duration = millTime;
	bsp_timer_array[timerId].callback = callback;
	bsp_timer_array[timerId].used = 1;
}

void bsp_stop_timer(int timerId)
{
	if(timerId >= BSP_TIMER_MAX)
		return;

	bsp_timer_array[timerId].used = 0;
	bsp_timer_array[timerId].callback = 0;
}

void bsp_timer_proc(void)
{
	int i;

	for(i=0; i<BSP_TIMER_MAX; i++)
	{
		if(bsp_timer_array[i].used)
		{
			if(HAL_GetTick() >= (bsp_timer_array[i].tick_satrt + bsp_timer_array[i].duration))
			{
				bsp_timer_array[i].tick_satrt = HAL_GetTick();
				if(bsp_timer_array[i].callback)
				{
					bsp_timer_array[i].callback(i);
				}
			}
		}
	}
}


/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(uint32_t ms)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0库版本
	if (SysTick_Config((SystemCoreClock/1000)*ms))	// ST3.5.0库版本SystemCoreClock/10不能超过16777216
	{ 
		/* Capture error */ 
		while (1);
	}
		// 关闭滴答定时器  
	ms_per_tick = ms;
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Enable(unsigned char enable)
{
	if(enable)
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	else
		SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;	
}

__weak void HAL_IncTick(void)
{
  uwTick += ms_per_tick;
  if(TickDelay > ms_per_tick)	
  	TickDelay-=ms_per_tick;
  else
  	TickDelay = 0;
}

__weak unsigned long long HAL_GetTick(void)
{
  return uwTick;
}


/*********************************************END OF FILE**********************/
