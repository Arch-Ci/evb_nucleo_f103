/**
  ******************************************************************************
  * @file    qmcX983.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    qmcX983驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "qmc6308.h"

#define QMC6308_LOG		printf

static unsigned char qmc6308_chipid = 0;
//static qmc6308_map g_map;
static uint8_t	qmc6308_addr = QMC6308_IIC_ADDR;
static uint8_t	qmc_chip = QMC_NONE;

static uint8_t qmc6308_read_block(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t ret = 0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_readreg(qmc6308_addr<<1, addr, data, len);
#else
		ret = I2C_BufferRead(qmc6308_addr<<1, addr, data, len);
#endif
	}

	return ret;
}

static uint8_t qmc6308_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t ret = 0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_writereg(qmc6308_addr<<1, addr, data);
#else
		ret = I2C_ByteWrite(qmc6308_addr<<1, addr, data); 
#endif
	}

	return ret;
}


void qmc6308_delay(int delay)
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

#if 0
static void qmc6308_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
}
#endif

int qmc6308_get_chipid(void)
{
	int ret = 0;
	int i;

	QMC6308_LOG("qmc6308_get_chipid addr=0x%x\n",qmc6308_addr);
	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &qmc6308_chipid , 1);
		QMC6308_LOG("qmc6308_get_chipid chipid = 0x%x\n", qmc6308_chipid);
		if(ret)
		{
			break;
		}
	}
	if(i>=10)
	{
		return 0;
	}
	if((qmc6308_chipid & 0xf0)==0)
	{
		return 0;
	}
	qmc_chip = QMC_6308;

	return 1;
}

static int qmc6310_get_chipid(void)
{
	int ret = 0;
	int i;

	QMC6308_LOG("qmc6310_get_chipid addr=0x%x\n",qmc6308_addr);
	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &qmc6308_chipid , 1);
		QMC6308_LOG("qmc6308_get_chipid chipid = 0x%x\n", qmc6308_chipid);
		if(ret)
		{
			break;
		}
	}
	if(i>=10)
	{
		return 0;
	}
	if(qmc6308_chipid != 0x80)
	{
		return 0;
	}
	qmc_chip = QMC_6310;

	return 1;
}

int qmc6308_read_mag_xyz(float *data)
{
	int res;
	unsigned char mag_data[6];
	short hw_d[3] = {0};
	float hw_f[3];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	while(!(rdy & 0x01) && (t1 < 5)){
		rdy = QMC6308_STATUS_REG;
		res = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
		t1++;
	}
	if(t1 >= 5)
	{
		return 0;
	}

	mag_data[0] = QMC6308_DATA_OUT_X_LSB_REG;
	res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(res == 0)
  	{
		return 0;
	}

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	hw_f[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	hw_f[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	hw_f[2] = (float)((float)hw_d[2] / 10.0f);		// ut

#if 0
	data[AXIS_X] = (float)(g_map.sign[AXIS_X]*hw_f[g_map.map[AXIS_X]]);
	data[AXIS_Y] = (float)(g_map.sign[AXIS_Y]*hw_f[g_map.map[AXIS_Y]]);
	data[AXIS_Z] = (float)(g_map.sign[AXIS_Z]*hw_f[g_map.map[AXIS_Z]]);
#else
	data[0] = hw_f[0];
	data[1] = hw_f[1];
	data[2] = hw_f[2];
#endif
	
	return res;
}


/* Set the sensor mode */
int qmc6308_set_mode(unsigned char mode)
{
	int err = 0;
	unsigned char ctrl1_value = 0;
	
	err = qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl1_value, 1);
	ctrl1_value = (ctrl1_value&(~0x03))|mode;
	QMC6308_LOG("qmc6308_set_mode, 0x%x = 0x%x \n", QMC6308_CTL_REG_ONE,ctrl1_value);
	err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, ctrl1_value);

	return err;
}

int qmc6308_set_output_data_rate(unsigned char rate){
	
	int err = 0;
	unsigned char ctrl1_value = 0;
	
	err = qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl1_value, 1);
	ctrl1_value = (ctrl1_value& (~0xE8)) | (rate << 5);
	QMC6308_LOG("qmc6308_set_output_data_rate, 0x%x = 0x%2x \n", QMC6308_CTL_REG_ONE,ctrl1_value);
	err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, ctrl1_value);

	return err;	
}

static int qmc6308_enable(void)
{
	int ret;

	if(qmc_chip == QMC_6308)
	{
		if(qmc6308_chipid == 0x80)
		{
			ret = qmc6308_write_reg(0x0d, 0x40);
			qmc6308_delay(10);
			ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
			qmc6308_delay(10);
			ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0xc3);
			qmc6308_delay(10);
		}
		else
		{
			ret = qmc6308_write_reg(0x0d, 0x40);
			qmc6308_delay(10);
			ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x08);
			qmc6308_delay(10);
			ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x63);
			qmc6308_delay(10);
		}
	}
	else
	{
		qmc6308_write_reg(0x0d, 0x40);
		qmc6308_delay(10);
		qmc6308_write_reg(0x29, 0x06);
		qmc6308_delay(10);
		//qmc6308_write_reg(0x0a, 0xCF);//0XA9=ODR =100HZ 0XA5 = 50HZ
		//qmc6308_write_reg(0x0a, 0x31);
		qmc6308_write_reg(0x0a, 0xc3);
		//qmc6308_write_reg(0x0a, 0x03);
		qmc6308_delay(10);
		qmc6308_write_reg(0x0b, 0x00); //30 GS
		//qmc6308_write_reg(0x0b, 0x02); //30 GS
		qmc6308_delay(10);
	}
	
	return ret;
}

static int qmc6308_disable(void)
{
	int ret;

	ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);

	return ret;
}

int qmc6308_do_selftest(float *data)
{
	float data1[3];
	float data2[3];
	int res;
	unsigned char mag_data[6];
	short hw_d[3] = {0};
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	//qmc6308_write_reg(0x0b, 0x00);
	qmc6308_write_reg(0x29, 0x06);
	qmc6308_write_reg(0x0a, 0x03);
	while(!(rdy & 0x01) && (t1 < 5)){
		rdy = QMC6308_STATUS_REG;
		res = qmc6308_read_block(QMC6308_STATUS_REG, &rdy, 1);
		t1++;
	}
	if(t1 >= 5)
	{
		return 0;
	}

	mag_data[0] = QMC6308_DATA_OUT_X_LSB_REG;
	res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(res == 0)
  	{
		return 0;
	}

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
	data1[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	data1[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	data1[2] = (float)((float)hw_d[2] / 10.0f);		// ut

	qmc6308_write_reg(0x0b, 0x40);
	qmc6308_delay(20);
	res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(res == 0)
  	{
		return 0;
	}
	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
	data2[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	data2[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	data2[2] = (float)((float)hw_d[2] / 10.0f);		// ut

	data[0] = data2[0]-data1[0];
	data[1] = data2[1]-data1[1];
	data[2] = data2[2]-data1[2];

	return 1;
}


static void qmc6308_dump_reg(void)
{
	unsigned char ctrl_value;
	unsigned char wafer_id;
	unsigned char die_id[2];

	qmc6308_read_block(0x37, &wafer_id, 1);
	QMC6308_LOG("qmc6308  wafer id:0x%x \n", wafer_id&0x1f);
	qmc6308_read_block(0x38, die_id, 2);
	QMC6308_LOG("qmc6308  die id:0x%x \n", (die_id[1]<<8)|die_id[0]);
	

	qmc6308_read_block(QMC6308_CTL_REG_ONE, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \n", QMC6308_CTL_REG_ONE, ctrl_value);
	qmc6308_read_block(QMC6308_CTL_REG_TWO, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \n", QMC6308_CTL_REG_TWO, ctrl_value);
	qmc6308_read_block(0x0d, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \n", 0x0d, ctrl_value);
	qmc6308_read_block(0x29, &ctrl_value, 1);
	QMC6308_LOG("qmc6308  0x%x=0x%x \n", 0x29, ctrl_value);
}

int qmc6308_init(void)
{
	int ret = 0;

	qmc6308_addr = QMC6308_IIC_ADDR;
	ret = qmc6308_get_chipid();
	if(!ret)
	{
		qmc6308_addr = QMC6310_IIC_ADDR_U;
		ret = qmc6310_get_chipid();
		if(!ret)
		{
			qmc6308_addr = QMC6310_IIC_ADDR_N;
			ret = qmc6310_get_chipid();
			if(!ret)
				return 0;
		}
	}
		
	//qmc6308_set_layout(1);
	qmc6308_disable();
	qmc6308_enable();
	qmc6308_dump_reg();

	return 1;
}

