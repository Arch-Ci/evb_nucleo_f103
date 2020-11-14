

#include "qmt200.h"

#define QMT200_LOG		printf

static unsigned char qmt200_chipid = 0;
static uint8_t	qmt200_addr = QMT200_IIC_ADDR;

static uint8_t qmt200_read_reg(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t ret = 0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qmt200_sw_readreg(qmt200_addr<<1, addr, data, len);
#else
		ret = I2C_BufferRead(qmt200_addr<<1, addr, data, len);
#endif
	}

	return ret;
}

static uint8_t qmt200_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t ret = 0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_writereg(qmt200_addr<<1, addr, data);
#else
		ret = I2C_ByteWrite(qmt200_addr<<1, addr, data); 
#endif
	}

	return ret;
}


void qmt200_delay(int delay)
{
	int i,j;
	for(i=0;i<delay;i++)
	{
		for(j=0;j<1000;j++)
		{
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
		}
	}
}

void qmt200_read_raw(void)
{
	uint8_t	reg_sensor[4];
	uint8_t	reg_temp[4];

	qmt200_read_reg(0xa2, reg_sensor, 4);
	qmt200_delay(10);
	qmt200_read_reg(0xa6, reg_temp, 4);
	QMT200_LOG("raw: sensor[0x%x 0x%x 0x%x 0x%x] temp[0x%x 0x%x 0x%x 0x%x]\n",
								reg_sensor[0],reg_sensor[1],reg_sensor[2],reg_sensor[3],
								reg_temp[0],reg_temp[1],reg_temp[2],reg_temp[3]);
}

int qmt200_init(void)
{
	int ret = 0;
	uint8_t	buf_reg[4];

	qmt200_addr = QMT200_IIC_ADDR;
	qmt200_read_reg(0x00, buf_reg, 3);
	qmt200_read_reg(0x01, buf_reg, 3);
	qmt200_read_reg(0x20, buf_reg, 3);

	return 1;
}

