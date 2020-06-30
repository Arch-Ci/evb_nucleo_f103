#ifndef __BSP_FLASH_H
#define	__BSP_FLASH_H

#include "stm32f10x.h"

/* STM32��������Ʒÿҳ��С2KByte���С�С������Ʒÿҳ��С1KByte */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)	//2048
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
#endif

//д�����ʼ��ַ�������ַ
#define WRITE_START_ADDR  ((uint32_t)0x0801FC00)
#define WRITE_END_ADDR    ((uint32_t)0x0801FFFF)

typedef enum 
{
	FAILED = 0, 
	PASSED = !FAILED
} BspFlashStatus;


//int InternalFlash_Test(void);
void flash_write_u16(void *buff, uint16_t len);
void flash_read(void *buff, uint16_t len);
void flash_reset(void);

#endif

