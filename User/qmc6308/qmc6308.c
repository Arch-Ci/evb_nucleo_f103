/**
  ******************************************************************************
  * @file    qmc6308.c
  * @author  QST0103 Corporation 
  * @version V1.0
  * @date    2020-01-01
  * @brief   qmc6308 driver
  * @Platform:STM32F103 NUCLEO 
  ******************************************************************************
  **/ 

#include "qmc6308.h"

#define QMC6308_LOG		printf
//static qmc6308_map g_map;
static qmc6308_data_t g_qmc6308;

static uint8_t qmc6308_read_block(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t ret = 0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SW_I2C)
		ret = qst_sw_readreg(g_qmc6308.slave_addr<<1, addr, data, len);
#else
		ret = I2C_BufferRead(g_qmc6308.slave_addr<<1, addr, data, len);
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
		ret = qst_sw_writereg(g_qmc6308.slave_addr<<1, addr, data);
#else
		ret = I2C_ByteWrite(g_qmc6308.slave_addr<<1, addr, data); 
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


#if defined(QMC6308_FILTER_SUPPORT)
void qmc6308_avg_filter_init(QmcAvgFilter_t *filter)
{
	filter->init = 0;
}

short qmc6308_avg_filter_run(QmcAvgFilter_t *filter, int in)
{
	int 	i;
	int 	sum = 0;
	short 	avg = 0;

	if(filter->init == 0)
	{
		for(i=0; i<AVG_FILTER_SIZE; i++)
		{
			filter->array[i] = in;
		}
		filter->init = 1;
	}

	sum = 0;
	for(i=1; i<AVG_FILTER_SIZE; i++)
	{
		sum += filter->array[i];
		filter->array[i-1] = filter->array[i];
	}
	filter->array[AVG_FILTER_SIZE-1] = in;
	sum += in;
	avg = (short)(sum/AVG_FILTER_SIZE);

	return avg;
}
#endif


int qmc6308_get_chipid(void)
{
	int ret = 0;
	int i;

	QMC6308_LOG("qmc6308_get_chipid addr=0x%x\n",g_qmc6308.slave_addr);
	g_qmc6308.chip_id = 0;
	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &g_qmc6308.chip_id, 1);
		QMC6308_LOG("qmc6308_get_chipid chipid = 0x%x\n", g_qmc6308.chip_id);
		if(ret)
		{
			break;
		}
	}
	if(g_qmc6308.chip_id == 0x80)
	{
		g_qmc6308.chip_type = QMC_6308;
		return 1;
	}
	else if(g_qmc6308.chip_id & 0xc0)
	{
		g_qmc6308.chip_type = QMC_6308_MPW;
		return 1;
	}
	else
	{
		return 0;
	}
}

static int qmc6310_get_chipid(void)
{
	int ret = 0;
	int i;

	QMC6308_LOG("qmc6310_get_chipid addr=0x%x\n",g_qmc6308.slave_addr);
	g_qmc6308.chip_id = 0;
	for(i=0; i<10; i++)
	{
		ret = qmc6308_read_block(QMC6308_CHIP_ID_REG, &g_qmc6308.chip_id , 1);
		QMC6308_LOG("qmc6308_get_chipid chipid = 0x%x\n", g_qmc6308.chip_id);
		if(ret)
		{
			break;
		}
	}
	if(g_qmc6308.chip_id == 0x80)
	{
		g_qmc6308.chip_type = QMC_6310;
		return 1;
	}
	else
	{
		return 0;
	}
}

int qmc6308_read_mag_xyz(float *data)
{
	int res;
	unsigned char mag_data[6];
	short hw_d[3] = {0};
	float hw_f[3];
	int t1 = 0;
	unsigned char rdy = 0;

	if(g_qmc6308.op_mode == QMC6308_SINGLE_MODE)
	{
		qmc6308_odr_mode(QMC6308_ODR_200HZ, QMC6308_SINGLE_MODE);
	}
	/* Check status register for data availability */
	while(!(rdy & 0x01) && (t1 < 5))
	{
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
#if defined(QMC6308_FILTER_SUPPORT)
	hw_d[0] = qmc6308_avg_filter_run(&g_qmc6308.avg[0], (int)hw_d[0]);
	hw_d[1] = qmc6308_avg_filter_run(&g_qmc6308.avg[1], (int)hw_d[1]);
	hw_d[2] = qmc6308_avg_filter_run(&g_qmc6308.avg[2], (int)hw_d[2]);
#endif
#if defined(QMC6308_MODE_SWITCH)
	if(g_qmc6308.set_ctl.mode == 0)
	{
		if((QMC6308_ABS(hw_d[0]) > 10000)||(QMC6308_ABS(hw_d[1]) > 10000))
		{
			g_qmc6308.set_ctl.count++;
			if(g_qmc6308.set_ctl.count >= 1)
			{
				//qmc6308_disable();
				//qmc6308_range_sr(QMC6308_RNG_30G, QMC6308_SET_ON);
				//qmc6308_odr_mode(QMC6308_ODR_CONTINUE, QMC6308_CONTINUE_MODE);
				qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);
				qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x01);
				qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0xc3);
				g_qmc6308.set_ctl.mode = 1;
				g_qmc6308.set_ctl.count = 0;
			}
		}
		else
		{
			g_qmc6308.set_ctl.count = 0;
		}
	}
	else
	{
		if((QMC6308_ABS(hw_d[0]) < 8000)&&(QMC6308_ABS(hw_d[1]) < 8000))
		{		
			g_qmc6308.set_ctl.count++;
			if(g_qmc6308.set_ctl.count >= 1)
			{
				//qmc6308_range_sr(QMC6308_RNG_30G, QMC6308_SET_RESET_ON);
				qmc6308_disable();
				qmc6308_enable();
				g_qmc6308.set_ctl.mode = 0;
				g_qmc6308.set_ctl.count = 0;
			}
		}
		else
		{
			g_qmc6308.set_ctl.count = 0;
		}
	}
#endif
	hw_f[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	hw_f[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	hw_f[2] = (float)((float)hw_d[2] / 10.0f);		// ut

	//data[AXIS_X] = (float)(g_map.sign[AXIS_X]*hw_f[g_map.map[AXIS_X]]);
	//data[AXIS_Y] = (float)(g_map.sign[AXIS_Y]*hw_f[g_map.map[AXIS_Y]]);
	//data[AXIS_Z] = (float)(g_map.sign[AXIS_Z]*hw_f[g_map.map[AXIS_Z]]);
	data[0] = hw_f[0];
	data[1] = hw_f[1];
	data[2] = hw_f[2];
	
	return res;
}


// set odr , mode,
int qmc6308_odr_mode(unsigned char odr, unsigned char mode)
{	
	int err = 0;
	unsigned char ctrl1_value = 0;

	g_qmc6308.op_mode = mode;
	ctrl1_value = (QMC6308_OSR2_8|QMC6308_OSR1_8|odr|mode);
	err = qmc6308_write_reg(QMC6308_CTL_REG_ONE, ctrl1_value);
	QMC6308_LOG("qmc6308_odr_mode, 0x%x = 0x%2x \n", QMC6308_CTL_REG_ONE,ctrl1_value);
	qmc6308_delay(1);

	return err;	
}

// set range , set/reset
int qmc6308_range_sr(unsigned char range, unsigned char set_reset)
{
	int err = 0;
	unsigned char ctrl2_value = 0;
	
	//err = qmc6308_read_block(QMC6308_CTL_REG_TWO, &ctrl2_value, 1);
	//ctrl2_value = ((ctrl2_value&0xf0)|range|set_reset);
	ctrl2_value = (range|set_reset);
	err = qmc6308_write_reg(QMC6308_CTL_REG_TWO, ctrl2_value);
	QMC6308_LOG("qmc6308_range_sr, 0x%x = 0x%x \n", QMC6308_CTL_REG_TWO,ctrl2_value);	
	qmc6308_delay(1);

	return err;
}

void qmc6308_soft_reset(void)
{
	qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x80);
	qmc6308_delay(5);
	qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x00);
}

static int qmc6308_enable(void)
{
	int ret;

	if(g_qmc6308.chip_type == QMC_6308)
	{
		ret = qmc6308_write_reg(0x0d, 0x40);
		qmc6308_delay(2);
		qmc6308_range_sr(QMC6308_RNG_30G, QMC6308_SET_RESET_ON);
		qmc6308_odr_mode(QMC6308_ODR_CONTINUE, QMC6308_CONTINUE_MODE);
	}
	else if(g_qmc6308.chip_type == QMC_6308_MPW)
	{
		ret = qmc6308_write_reg(0x0d, 0x40);
		qmc6308_delay(1);
		ret = qmc6308_write_reg(QMC6308_CTL_REG_TWO, 0x08);
		qmc6308_delay(1);
		ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x63);
		qmc6308_delay(1);
	}
	else
	{
		qmc6308_write_reg(0x0d, 0x40);
		qmc6308_delay(1);
		qmc6308_write_reg(0x0a, 0xc3);
		qmc6308_delay(1);
		qmc6308_write_reg(0x0b, 0x00);
		qmc6308_delay(1);
	}
	
	return ret;
}

static int qmc6308_disable(void)
{
	int ret;

	ret = qmc6308_write_reg(QMC6308_CTL_REG_ONE, 0x00);

	return ret;
}

// 80-120ut, z=1/3 xy
int qmc6308_do_selftest(short *data)
{
	short data1[3];
	short data2[3];
	int res;
	unsigned char mag_data[6];
	//short hw_d[3] = {0};
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	//qmc6308_write_reg(0x0b, 0x00);
	//qmc6308_write_reg(0x29, 0x06);
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

	data1[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	data1[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	data1[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
	//data1[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	//data1[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	//data1[2] = (float)((float)hw_d[2] / 10.0f);		// ut

	qmc6308_write_reg(0x0b, 0x40);
	qmc6308_delay(100);
	res = qmc6308_read_block(QMC6308_DATA_OUT_X_LSB_REG, mag_data, 6);
	if(res == 0)
  	{
		return 0;
	}
	data2[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	data2[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	data2[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
	//data2[0] = (float)((float)hw_d[0] / 10.0f);		// ut
	//data2[1] = (float)((float)hw_d[1] / 10.0f);		// ut
	//data2[2] = (float)((float)hw_d[2] / 10.0f);		// ut

	data[0] = QMC6308_ABS(data2[0]-data1[0]);
	data[1] = QMC6308_ABS(data2[1]-data1[1]);
	data[2] = QMC6308_ABS(data2[2]-data1[2]);

	qmc6308_disable();
	qmc6308_enable();
	
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
}

int qmc6308_init(void)
{
	int ret = 0;

	g_qmc6308.slave_addr = QMC6308_IIC_ADDR;
	ret = qmc6308_get_chipid();
	if(!ret)
	{
		g_qmc6308.slave_addr = QMC6310_IIC_ADDR_U;
		ret = qmc6310_get_chipid();
		if(!ret)
		{
			g_qmc6308.slave_addr = QMC6310_IIC_ADDR_N;
			ret = qmc6310_get_chipid();
			if(!ret)
				return 0;
		}
	}
		
	qmc6308_soft_reset();
	//qmc6308_set_layout(1);
	qmc6308_disable();
	qmc6308_enable();
	qmc6308_dump_reg();
#if defined(QMC6308_FILTER_SUPPORT)
	qmc6308_avg_filter_init(&g_qmc6308.avg[0]);
	qmc6308_avg_filter_init(&g_qmc6308.avg[1]);
	qmc6308_avg_filter_init(&g_qmc6308.avg[2]);
#endif
#if defined(QMC6308_MODE_SWITCH)
	g_qmc6308.set_ctl.mode = 0;
	g_qmc6308.set_ctl.count = 0;
#endif

	return 1;
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

