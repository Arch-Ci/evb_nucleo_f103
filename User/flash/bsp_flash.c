/**
  ******************************************************************************
  * @file    bsp_internalFlash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �ڲ�FLASH��д���Է���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 ָ���� ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_flash.h"   
#include "bsp_usart.h"

#if 0
int InternalFlash_Test(void)
{
	uint32_t EraseCounter = 0x00; 	//��¼Ҫ��������ҳ
	uint32_t Address = 0x00;				//��¼д��ĵ�ַ
	uint32_t Data = 0x3210ABCD;			//��¼д�������
	uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ
	
	FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��	
	BspFlashStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
	

  /* ���� */
  FLASH_Unlock();

  /* ����Ҫ��������ҳ */
  NbrOfPage = (WRITE_END_ADDR - WRITE_START_ADDR) / FLASH_PAGE_SIZE;

  /* ������б�־λ */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /* ��ҳ����*/
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
  
	}
  
  /* ���ڲ�FLASHд������ */
  Address = WRITE_START_ADDR;

  while((Address < WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  {
    FLASHStatus = FLASH_ProgramWord(Address, Data);
    Address = Address + 4;
  }

  FLASH_Lock();
  
  /* ���д��������Ƿ���ȷ */
  Address = WRITE_START_ADDR;

  while((Address < WRITE_END_ADDR) && (MemoryProgramStatus != FAILED))
  {
    if((*(__IO uint32_t*) Address) != Data)
    {
      MemoryProgramStatus = FAILED;
    }
    Address += 4;
  }
	return MemoryProgramStatus;
}
#endif

void flash_write_u16(void *buff, uint16_t len)
{
	uint16_t  i;
	uint16_t  *buff_u16 = (uint16_t*)buff;

	FLASH_Status FLASHStatus = FLASH_COMPLETE;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR);

	if(FLASHStatus == FLASH_COMPLETE)
	{
		for(i=0; i<((len+1)/2); i++)
		{
				FLASHStatus = FLASH_ProgramHalfWord(WRITE_START_ADDR+(i*sizeof(uint16_t)), buff_u16[i]);
		}
		FLASH_Lock();
	}
}

void flash_write_u32(void *buff, uint32_t len)
{
	uint16_t  i;
	uint32_t  *buff_u32 = (uint32_t*)buff;

	FLASH_Status FLASHStatus = FLASH_COMPLETE;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR);

	if(FLASHStatus == FLASH_COMPLETE)
	{
		for(i=0; i<((len+1)/2); i++)
		{
				FLASHStatus = FLASH_ProgramWord(WRITE_START_ADDR+(i*sizeof(uint32_t)), buff_u32[i]);
		}
		FLASH_Lock();
	}
}

void flash_read(void *buff, uint16_t len)
{
	uint16_t i;
	uint8_t *buff_u8 = (uint8_t *)buff;
	for(i=0; i<len; i++)
	{
		buff_u8[i] = *((__IO uint8_t*)(WRITE_START_ADDR+i));
	}
}

void flash_reset(void)
{
	FLASH_ErasePage(WRITE_START_ADDR);
}

