
#include "px318j.h"
#include <math.h>

#define PX318J_INT_PUSHPULL

static void px318j_delay(unsigned int delay)
{
	qst_delay(delay);
}

uint8_t px318j_write_reg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat)
{
	unsigned char ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_writereg(slave, reg_add, reg_dat);
#else
		ret = I2C_ByteWrite(slave, reg_add, reg_dat); 
#endif
	}

	return ret;
}

uint8_t px318j_write_word(uint8_t slave, uint8_t reg_add,uint16_t reg_dat)
{
	unsigned char ret = 0;

	ret = px318j_write_reg(slave, reg_add, (uint8_t)(reg_dat&0xff));
	ret = px318j_write_reg(slave, reg_add+1, (uint8_t)((reg_dat>>8)&0xff));
	
	return ret;
}

uint8_t px318j_read_reg(uint8_t slave, uint8_t reg_add,unsigned char* Read,uint8_t num)
{
	unsigned char ret = 0;
	int retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_readreg(slave, reg_add, Read, num);
#else
		ret = I2C_BufferRead(slave, reg_add, Read, num);
#endif
	}

	return ret;
}

uint8_t px318j_set_threshold(uint16_t high, uint16_t low)
{
	uint8_t ret = 0;
	
	px318j_write_word(PX318J_II_ADDR, 0x6C, low); //�趨����ֵ
	px318j_write_word(PX318J_II_ADDR, 0x6E, high); //�趨����ֵ

	return ret;
}

uint16_t px318j_read_proximity(void)
{
	uint8_t	value[2];
	uint16_t distance = 0;

	px318j_read_reg(PX318J_II_ADDR, 0x00, value, 2);
	distance = (uint16_t)(((uint16_t)(value[1]<<8))|value[0]);

	px318j_delay(30);

	return distance;
}

uint8_t px318j_get_int_flag(void)
{
	uint8_t intflag = 0;

	px318j_read_reg(PX318J_II_ADDR, 0xFE, &intflag, 1);
	if(intflag & 0x02)
	{
		px318j_write_reg(PX318J_II_ADDR, 0xFE, 0x00); //����Д���ˣ���Ҫ����� POR ��ˡ�
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t px318j_init(void)
{
	uint8_t value = 0;
	uint8_t ret;

	PX318J_LOG("px318j_init \n");
	ret = px318j_write_reg(PX318J_II_ADDR, 0xF4, 0xEE); //������
	if(!ret)	return 0;
	px318j_delay(50); 							  //������������ӳ�30��������
#if defined(PX318J_INT_PUSHPULL)
	px318j_read_reg(PX318J_II_ADDR, 0xf1, &value, 1); //
	px318j_write_reg(PX318J_II_ADDR, 0xf1, value|0x80); //0xf1 bit7 set 1
#endif
	px318j_write_reg(PX318J_II_ADDR, 0x60, 0x15); //PsData Ϊ 10 λ�޷���������Ӧʱ��Ϊ30���롣
	px318j_write_reg(PX318J_II_ADDR, 0x61, 0x20); //������Ϊ 32W = 64 ΢��
	px318j_write_reg(PX318J_II_ADDR, 0x64, 0x0B); //��������Ϊ 12 ����
	px318j_write_word(PX318J_II_ADDR, 0x6C, 100); //�趨����ֵ
	px318j_write_word(PX318J_II_ADDR, 0x6E, 300); //�趨����ֵ
	px318j_write_reg(PX318J_II_ADDR, 0xFE, 0x00); //����жϱ�־����Ҫ�����POR��־��
	px318j_write_reg(PX318J_II_ADDR, 0xF0, 0x02); //����������
	px318j_delay(20); 							  //����������ӳ� 10 ����
#if 0
	px318j_read_reg(PX318J_II_ADDR, 0x60, &value, 1);
	printf("px318j 0x60=0x%x \n", value);
	px318j_read_reg(PX318J_II_ADDR, 0x61, &value, 1);
	printf("px318j 0x61=0x%x \n", value);
	px318j_read_reg(PX318J_II_ADDR, 0x64, &value, 1);
	printf("px318j 0x64=0x%x \n", value);
#endif

	return 1;
}


