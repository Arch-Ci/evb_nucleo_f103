/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
  *          ���õ��� 1us 10us 1ms �жϡ�     
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
//SysTick_Config��Ҫ���������ж�����������STK_VAL�Ĵ���������SysTickʱ��ΪAHB 
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
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
void SysTick_Init(uint32_t ms)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0��汾
	if (SysTick_Config((SystemCoreClock/1000)*ms))	// ST3.5.0��汾SystemCoreClock/10���ܳ���16777216
	{ 
		/* Capture error */ 
		while (1);
	}
		// �رյδ�ʱ��  
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
