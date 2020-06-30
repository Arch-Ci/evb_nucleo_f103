
#include "qmi8610.h"
#include <math.h>

//#define FIS210X_SLAVE_ADDR		0xd4		//(0x6a<<1)
#define FIS210X_SLAVE_ADDR			0xd6		//(0x6b<<1)
#define qst_printf					printf

//#define QMI8610_UINT_MG_DPS

enum
{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,

	AXIS_TOTAL
};

typedef struct 
{
	short 				sign[AXIS_TOTAL];
	unsigned short 		map[AXIS_TOTAL];
}qst_imu_layout;

static uint16_t acc_lsb_div = 0;
static uint16_t gyro_lsb_div = 0;
static struct FisImuConfig fisimu_config;

#if defined(QMI8610_USE_FIFO)
static uint8_t m_fifoMode = 0;
static uint8_t fifo_level = 0;
static uint8_t fifo_buf[12*96];
#endif
uint8_t qmi8610_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t ret=0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_fisimu_spi_write(reg,value);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_writereg(FIS210X_SLAVE_ADDR, reg, value);
#else
		ret = I2C_ByteWrite(FIS210X_SLAVE_ADDR, reg,value);
#endif
	}
	return ret;
}

uint8_t qmi8610_write_regs(uint8_t reg, uint8_t *value, uint8_t len)
{
	uint8_t ret=0;	
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_fisimu_spi_write_bytes(reg, value, len);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_writeregs(FIS210X_SLAVE_ADDR, reg, value, len);
#else
		ret = I2C_BufferRead(FIS210X_SLAVE_ADDR, reg, value, len);
#endif
	}
	return ret;
}

uint8_t qmi8610_read_reg(uint8_t reg, uint8_t* buf, uint16_t len)
{
	uint8_t ret=0;
	uint32_t retry = 0;

	while((!ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_8610_spi_read(reg, buf, len);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_readreg(FIS210X_SLAVE_ADDR, reg, buf, len);
#else
		ret = I2C_BufferRead(FIS210X_SLAVE_ADDR, reg, buf,  len);
#endif
	}
	return ret;
}

#if 0
static qst_imu_layout imu_map;

void qmi8610_set_layout(short layout)
{
	if(layout == 0)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
}
#endif

void qmi8610_config_acc(enum qmi8610_AccRange range, enum qmi8610_AccOdr odr, enum qmi8610_LpfConfig lpfEnable, enum qmi8610_HpfConfig hpfEnable)
{
	unsigned char ctl_dada;
	unsigned char range_set;

	switch (range)
	{
		case AccRange_2g:
			range_set = 0<<3;
			acc_lsb_div = (1<<14);
			break;
		case AccRange_4g:
			range_set = 1<<3;
			acc_lsb_div = (1<<13);
			break;
		case AccRange_8g:
			range_set = 2<<3;
			acc_lsb_div = (1<<12);
			break;
		default: 
			range_set = 2<<3;
			acc_lsb_div = (1<<12);
	}
	ctl_dada = (unsigned char)range_set|(unsigned char)odr;
	qmi8610_write_reg(Qmi8610Register_Ctrl2, ctl_dada);
// set LPF & HPF
	qmi8610_read_reg(Qmi8610Register_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xfc;
	if(lpfEnable == Lpf_Enable)
		ctl_dada |=0x02;
	if(hpfEnable == Hpf_Enable)
		ctl_dada |=0x01;
	qmi8610_write_reg(Qmi8610Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void qmi8610_config_gyro(enum qmi8610_GyrRange range, enum qmi8610_GyrOdr odr, enum qmi8610_LpfConfig lpfEnable, enum qmi8610_HpfConfig hpfEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada; 
	ctl_dada = (unsigned char)range | (unsigned char)odr;
	qmi8610_write_reg(Qmi8610Register_Ctrl3, ctl_dada);

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case GyrRange_32dps:
			gyro_lsb_div = 1024;
			break;
		case GyrRange_64dps:
			gyro_lsb_div = 512;
			break;
		case GyrRange_128dps:
			gyro_lsb_div = 256;
			break;
		case GyrRange_256dps:
			gyro_lsb_div = 128;
			break;
		case GyrRange_512dps:
			gyro_lsb_div = 64;
			break;
		case GyrRange_1024dps:
			gyro_lsb_div = 32;
			break;
		case GyrRange_2048dps:
			gyro_lsb_div = 16;
			break;
		case GyrRange_2560dps:
			//gyro_lsb_div = 8;
			break;
		default: 
			gyro_lsb_div = 32;
			break;
	}

// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	qmi8610_read_reg(Qmi8610Register_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xf3;
	if(lpfEnable == Lpf_Enable)
		ctl_dada |=0x08;
	if(hpfEnable == Hpf_Enable)
		ctl_dada |=0x04;
	qmi8610_write_reg(Qmi8610Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void qmi8610_config_ae(enum qmi8610_AeOdr odr)
{
	// Configure Accelerometer and Gyroscope settings
	qmi8610_config_acc(AccRange_8g, AccOdr_1024Hz, Lpf_Enable, Hpf_Disable);
	qmi8610_config_gyro(GyrRange_2048dps, GyrOdr_1024Hz, Lpf_Enable, Hpf_Disable);
	qmi8610_write_reg(Qmi8610Register_Ctrl6, odr);
}

void qmi8610_config_mag(enum qmi8610_MagDev device, enum qmi8610_MagOdr odr)
{
	
}

uint8_t qmi8610_readStatus0(void)
{
	uint8_t status;

	qmi8610_read_reg(Qmi8610Register_Status0, &status, sizeof(status));

	return status;
}
/*!
 * \brief Blocking read of data status register 1 (::Qmi8610Register_Status1).
 * \returns Status byte \see STATUS1 for flag definitions.
 */
uint8_t qmi8610_readStatus1(void)
{
	uint8_t status;
	
	qmi8610_read_reg(Qmi8610Register_Status1, &status, sizeof(status));

	return status;
}

int8_t qmi8610_readTemp(void)
{
	int8_t temp;
	qmi8610_read_reg(Qmi8610Register_Temperature, (uint8_t*)&temp, sizeof(temp));
	return temp;
}

void qmi8610_read_acc_xyz(float acc_xyz[3])
{
	unsigned char	buf_reg[6];
	short 			raw_acc_xyz[3];

#if defined(QST_USE_SPI)
	qmi8610_read_reg(Qmi8610Register_Ax_L, &buf_reg[0], 6); 	// 0x19, 25
#else
	qmi8610_read_reg(Qmi8610Register_Ax_L|0x80, buf_reg, 6); 	// 0x19, 25
#endif
	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));
//	if(cali_flag == FALSE)
//	{
//		raw_acc_xyz[0]+=offset[0];
//		raw_acc_xyz[1]+=offset[1];
//		raw_acc_xyz[2]+=offset[2];
//	}

	acc_xyz[0] = (raw_acc_xyz[0]*ONE_G)/acc_lsb_div;
	acc_xyz[1] = (raw_acc_xyz[1]*ONE_G)/acc_lsb_div;
	acc_xyz[2] = (raw_acc_xyz[2]*ONE_G)/acc_lsb_div;
//	acc_layout = 0;
//	acc_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*acc_xyz[0];
//	acc_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*acc_xyz[1];
//	acc_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*acc_xyz[2];
	qst_printf("fis210x acc:	%f	%f	%f\n", acc_xyz[0], acc_xyz[1], acc_xyz[2]);
}

void qmi8610_read_gyro_xyz(float gyro_xyz[3])
{
	unsigned char	buf_reg[6];
	short 			raw_gyro_xyz[3];

#if defined(QST_USE_SPI)
	qmi8610_read_reg(Qmi8610Register_Gx_L, &buf_reg[0], 6); 	// 0x1f, 31
#else
	qmi8610_read_reg(Qmi8610Register_Gx_L|0x80, buf_reg, 6);  	// 0x1f, 31
#endif
	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));	
//	if(cali_flag == FALSE)
//	{
//		raw_gyro_xyz[0] += offset_gyro[0];
//		raw_gyro_xyz[1] += offset_gyro[1];
//		raw_gyro_xyz[2] += offset_gyro[2];
//	}

	gyro_xyz[0] = (raw_gyro_xyz[0]*1.0f)/gyro_lsb_div;
	gyro_xyz[1] = (raw_gyro_xyz[1]*1.0f)/gyro_lsb_div;
	gyro_xyz[2] = (raw_gyro_xyz[2]*1.0f)/gyro_lsb_div;
//	acc_layout = 0;
//	gyro_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*gyro_xyz[0];
//	gyro_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*gyro_xyz[1];
//	gyro_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*gyro_xyz[2];
	qst_printf("fis210x gyro:	%f	%f	%f\n", gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]);
}

void qmi8610_read_xyz(float acc[3], float gyro[3], unsigned char *tim_count)
{
	unsigned char	buf_reg[12];
	short 			raw_acc_xyz[3];
	short 			raw_gyro_xyz[3];
//	float acc_t[3];
//	float gyro_t[3];

	if(tim_count)
	{
		qmi8610_read_reg(Qmi8610Register_CountOut, tim_count, 1);	// 0x18	24
	}
	qmi8610_read_reg(Qmi8610Register_Ax_L|0x80, buf_reg, 12); 	// 0x19, 25
	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	raw_gyro_xyz[0] = (short)((buf_reg[7]<<8) |( buf_reg[6]));
	raw_gyro_xyz[1] = (short)((buf_reg[9]<<8) |( buf_reg[8]));
	raw_gyro_xyz[2] = (short)((buf_reg[11]<<8) |( buf_reg[10]));	

#if defined(QMI8610_UINT_MG_DPS)
	// mg
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X]*1000.0f)/acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y]*1000.0f)/acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z]*1000.0f)/acc_lsb_div;
#else
	// m/s2
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X]*ONE_G)/acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y]*ONE_G)/acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z]*ONE_G)/acc_lsb_div;
#endif
//	acc[AXIS_X] = imu_map.sign[AXIS_X]*acc_t[imu_map.map[AXIS_X]];
//	acc[AXIS_Y] = imu_map.sign[AXIS_Y]*acc_t[imu_map.map[AXIS_Y]];
//	acc[AXIS_Z] = imu_map.sign[AXIS_Z]*acc_t[imu_map.map[AXIS_Z]];

#if defined(QMI8610_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0]*1.0f)/gyro_lsb_div;
	gyro[1] = (float)(raw_gyro_xyz[1]*1.0f)/gyro_lsb_div;
	gyro[2] = (float)(raw_gyro_xyz[2]*1.0f)/gyro_lsb_div;
#else
	// rad/s
	gyro[AXIS_X] = (float)(raw_gyro_xyz[AXIS_X]*0.01745f)/gyro_lsb_div;		// *pi/180
	gyro[AXIS_Y] = (float)(raw_gyro_xyz[AXIS_Y]*0.01745f)/gyro_lsb_div;
	gyro[AXIS_Z] = (float)(raw_gyro_xyz[AXIS_Z]*0.01745f)/gyro_lsb_div;
#endif	
//	gyro[AXIS_X] = imu_map.sign[AXIS_X]*gyro_t[imu_map.map[AXIS_X]];
//	gyro[AXIS_Y] = imu_map.sign[AXIS_Y]*gyro_t[imu_map.map[AXIS_Y]];
//	gyro[AXIS_Z] = imu_map.sign[AXIS_Z]*gyro_t[imu_map.map[AXIS_Z]];
	//printf("%f	%f	%f\n",gyro[0],gyro[1],gyro[2]);
}


void qmi8610_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned char *tim_count)
{
	unsigned char	buf_reg[12];

	if(tim_count)
	{
		qmi8610_read_reg(Qmi8610Register_CountOut, tim_count, 1);	// 0x18	24
	}
	qmi8610_read_reg(Qmi8610Register_Ax_L|0x80, buf_reg, 12); 	// 0x19, 25

	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	raw_gyro_xyz[0] = (short)((buf_reg[7]<<8) |( buf_reg[6]));
	raw_gyro_xyz[1] = (short)((buf_reg[9]<<8) |( buf_reg[8]));
	raw_gyro_xyz[2] = (short)((buf_reg[11]<<8) |( buf_reg[10]));
}


// for XKF3
static inline void applyScaleFactor(float scaleFactor, uint8_t nElements, uint8_t const* rawData, float* calibratedData)
{
	for (int i = 0; i < nElements; ++i)
	{
		calibratedData[i] = scaleFactor * (int16_t)((uint16_t)rawData[2 * i] |((uint16_t)rawData[2 * i + 1] << 8));
	}
}

void qmi8610_processAccelerometerData(uint8_t const* rawData, float* calibratedData)
{
	applyScaleFactor((float)(ONE_G/acc_lsb_div), 3, rawData, calibratedData);
}

void qmi8610_processGyroscopeData(uint8_t const* rawData, float* calibratedData)
{
	applyScaleFactor((float)(M_PI/(gyro_lsb_div*180)), 3, rawData, calibratedData);
}

void qmi8610_read_rawsample(struct Qmi8610RawSample *sample)
{
	qmi8610_read_reg(Qmi8610Register_CountOut, &(sample->sampleCounter), sizeof(uint8_t));
	sample->accelerometerData = sample->sampleBuffer;
	sample->gyroscopeData = sample->sampleBuffer+QMI8610_SAMPLE_SIZE;
	qmi8610_read_reg(Qmi8610Register_Ax_L, (uint8_t*)sample->accelerometerData, QMI8610_SAMPLE_SIZE);
	qmi8610_read_reg(Qmi8610Register_Gx_L, (uint8_t*)sample->gyroscopeData, QMI8610_SAMPLE_SIZE);
}

static void writeCalibrationVectorBuffer(float const* calVector, float conversionFactor, uint8_t fractionalBits)
{
	int i;
	int16_t o;
	uint8_t calCmd[6];
	//calCmd[0] = Qmi8610Register_Cal1_L;

	for(i = 0; i < 3; ++i)
	{
		o = (int16_t)roundf(calVector[i] * conversionFactor * (1 << fractionalBits));
		calCmd[(2 * i)] = o & 0xFF;
		calCmd[(2 * i) + 1] = o >> 8;
	}
	qmi8610_write_regs(Qmi8610Register_Cal1_L, calCmd, sizeof(calCmd));
}

void qmi8610_doCtrl9Command(enum qmi8610_Ctrl9Command cmd)
{
	//uint8_t gyroConfig;
	//const uint8_t oisModeBits = 0x06;
	//unsigned char oisEnabled;
	uint8_t status = 0;
	uint32_t count = 0;

	//qmi8610_read_reg(Qmi8610Register_Ctrl3, &gyroConfig, sizeof(gyroConfig));
	//oisEnabled = ((gyroConfig & oisModeBits) == oisModeBits);
	//if(oisEnabled)
	//{
	//	qmi8610_write_reg(Qmi8610Register_Ctrl3, (gyroConfig & ~oisModeBits));
	//}

	//g_fisDriverHal->waitForEvent(Fis_Int1, FisInt_low);
	//qst_delay(300);

	qmi8610_write_reg(Qmi8610Register_Ctrl9, cmd);
	//g_fisDriverHal->waitForEvent(Fis_Int1, FisInt_high);
	//qst_delay(300);

	// Check that command has been executed
	while(((status & QMI8610_STATUS1_CMD_DONE)==0)&&(count<1000))
	{
		qmi8610_read_reg(Qmi8610Register_Status1, &status, sizeof(status));
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
		count++;
	}

	printf("count = %d\n", count);
	//assert(status & QMI8610_STATUS1_CMD_DONE);

	//g_fisDriverHal->waitForEvent(Fis_Int1, FisInt_low);
	//qst_delay(300);

	//if(oisEnabled)
	//{
		// Re-enable OIS mode configuration if necessary		
	//	qmi8610_write_reg(Qmi8610Register_Ctrl3, gyroConfig);
	//}
}

void qmi8610_applyAccelerometerOffset(float const* offset, enum qmi8610_AccUnit unit)
{
	const float conversionFactor = (unit == AccUnit_ms2) ? (1 / ONE_G) : 1;
	writeCalibrationVectorBuffer(offset, conversionFactor, 11);
	qmi8610_doCtrl9Command(Ctrl9_SetAccelOffset);
}

void qmi8610_applyGyroscopeOffset(float const* offset, enum qmi8610_GyrUnit unit)
{
	const float conversionFactor = (unit == GyrUnit_rads) ? 180 / M_PI : 1;
	writeCalibrationVectorBuffer(offset, conversionFactor, 6);
	qmi8610_doCtrl9Command(Ctrl9_SetGyroOffset);
}

void qmi8610_applyOffsetCalibration(struct qmi8610_offsetCalibration const* cal)
{
   qmi8610_applyAccelerometerOffset(cal->accOffset, cal->accUnit);
   qmi8610_applyGyroscopeOffset(cal->gyrOffset, cal->gyrUnit);
}

// for XKF3

void qmi8610_enableWakeOnMotion(void)
{
	uint8_t womCmd[3];
	enum qmi8610_Interrupt interrupt = Fis_Int1;
	enum qmi8610_InterruptInitialState initialState = InterruptInitialState_low;
	enum qmi8610_WakeOnMotionThreshold threshold = WomThreshold_low;
	uint8_t blankingTime = 0x00;
	const uint8_t blankingTimeMask = 0x3F;

	qmi8610_enableSensors(QMI8610_CTRL7_DISABLE_ALL);
	qmi8610_config_acc(AccRange_8g, AccOdr_256Hz, Lpf_Enable, Hpf_Disable);
	//qmi8610_config_acc(AccRange_2g, AccOdr_LowPower_25Hz, Lpf_Disable, Hpf_Disable);
	qmi8610_config_gyro(GyrRange_1024dps, GyrOdr_256Hz, Lpf_Enable, Hpf_Disable);

	womCmd[0] = Qmi8610Register_Cal1_L;		//WoM Threshold: absolute value in mg (with 1mg/LSB resolution)
	womCmd[1] = threshold;
	womCmd[2] = (uint8_t)interrupt | (uint8_t)initialState | (blankingTime & blankingTimeMask);
	qmi8610_write_reg(Qmi8610Register_Cal1_L, womCmd[1]);
	qmi8610_write_reg(Qmi8610Register_Cal1_H, womCmd[2]);

	qmi8610_doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
	qmi8610_enableSensors(QMI8610_CONFIG_ACC_ENABLE);
}

void qmi8610_disableWakeOnMotion(void)
{
	qmi8610_enableSensors(QMI8610_CTRL7_DISABLE_ALL);
	qmi8610_write_reg(Qmi8610Register_Cal1_L, 0);
	qmi8610_doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
}


#if defined(QMI8610_USE_FIFO)
uint16_t qmi8610_configureFifo(enum qmi8610_FifoWatermarkLevel watermark, enum qmi8610_FifoSize size, enum qmi8610_FifoMode mode)
{
	uint16_t fifoSize = 0;
	uint16_t watermarkLevel = 0;

	m_fifoMode = (uint8_t)watermark | (uint8_t)size | (uint8_t)mode;
	qmi8610_doCtrl9Command(Ctrl9_ResetFifo);
	qmi8610_write_reg(Qmi8610Register_FifoCtrl, m_fifoMode);

	switch(size)
	{
		case FifoSize_16:
			fifoSize = 16;
			break;
		case FifoSize_32:
			fifoSize = 32;
			break;
		case FifoSize_64:
			fifoSize = 64;
			break;
		case FifoSize_128:
			fifoSize = 128;
			break;
	}
	switch (watermark)
	{
		//! \todo Check if this should be 0 or 1
		case Fifo_WatermarkEmpty:
			watermarkLevel = 0;
			break;
		case Fifo_WatermarkOneQuarter:
			watermarkLevel = fifoSize / 4;
			break;
		case Fifo_WatermarkHalf:
			watermarkLevel = fifoSize / 2;
			break;
		case Fifo_WatermarkThreeQuarters:
			watermarkLevel = (fifoSize * 3) / 4;
			break;
	}

	if(mode == FifoMode_Disable)
		return 0;
	else
		return watermarkLevel;
}


void qmi8610_readFifo(uint8_t* data, uint16_t dataLength)
{
	qmi8610_doCtrl9Command(Ctrl9_ReadFifo);	
	qmi8610_read_reg(Qmi8610Register_FifoCtrl, &m_fifoMode, sizeof(m_fifoMode));
	qmi8610_write_reg(Qmi8610Register_FifoCtrl, m_fifoMode|0x80);
	qmi8610_read_reg(Qmi8610Register_FifoData|0x80, data, dataLength);
	qmi8610_write_reg(Qmi8610Register_FifoCtrl, m_fifoMode);
}


void qmi8610_read_fifo(void)
{
	int i = 0;

	qmi8610_readFifo(fifo_buf, fifo_level*12);
	for(i=0; i<fifo_level; i++)
	{
		short raw[6];
		raw[0] = (fifo_buf[i*12+1]<<8)|(fifo_buf[i*12+0]);
		raw[1] = (fifo_buf[i*12+3]<<8)|(fifo_buf[i*12+2]);
		raw[2] = (fifo_buf[i*12+5]<<8)|(fifo_buf[i*12+4]);
		raw[3] = (fifo_buf[i*12+7]<<8)|(fifo_buf[i*12+6]);
		raw[4] = (fifo_buf[i*12+9]<<8)|(fifo_buf[i*12+8]);
		raw[5] = (fifo_buf[i*12+11]<<8)|(fifo_buf[i*12+10]);
		qst_printf("%d-- %d %d %d    %d %d %d\n", i, raw[0],raw[1],raw[2],raw[3],raw[4],raw[5]);
	}
}
#endif

void qmi8610_enableSensors(uint8_t enableFlags)
{
	if(enableFlags & QMI8610_CONFIG_AE_ENABLE)
	{
		enableFlags |= QMI8610_CTRL7_ACC_ENABLE | QMI8610_CTRL7_GYR_ENABLE;
	}

	qmi8610_write_reg(Qmi8610Register_Ctrl7, enableFlags & QMI8610_CTRL7_ENABLE_MASK);
}


void qmi8610_Config_apply(struct FisImuConfig const* config)
{
	uint8_t fisSensors = config->inputSelection;

	if(fisSensors & QMI8610_CONFIG_AE_ENABLE)
	{
		qmi8610_config_ae(config->aeOdr);
	}
	else
	{
		if (config->inputSelection & QMI8610_CONFIG_ACC_ENABLE)
		{
			qmi8610_config_acc(config->accRange, config->accOdr, Lpf_Enable, Hpf_Disable);
		}
		if (config->inputSelection & QMI8610_CONFIG_GYR_ENABLE)
		{
			qmi8610_config_gyro(config->gyrRange, config->gyrOdr, Lpf_Enable, Hpf_Disable);
		}
	}

	if(config->inputSelection & QMI8610_CONFIG_MAG_ENABLE)
	{
		qmi8610_config_mag(config->magDev, config->magOdr);
	}
	qmi8610_enableSensors(fisSensors);	
}

void qmi8610_serial_num(void)
{
	uint8_t reading[13];
	int32_t UID, chipId, MEMS_ID;

	qmi8610_read_reg(0x2D|0x80, reading, 13);
	chipId = (int)((((int)reading[7] & 0x07) << 24) + ((int)reading[6] << 16) + ((int)reading[5] << 8) + reading[4]);

	UID = (int)(((int)reading[3] << 24) + ((int)reading[2] << 16) + ((int)reading[1] << 8) + reading[0]);
	chipId = (int)((((int)reading[7] & 0x07) << 24) + ((int)reading[6] << 16) + ((int)reading[5] << 8) + reading[4]);
	MEMS_ID = (int)(reading[11]);

	printf("\nINFO: MEMS_ID number %d was detected" ,MEMS_ID);
	printf("\nINFO: UID number %d was detected" ,UID);
	printf("\nINFO: chipId number %d was detected" ,chipId);
	printf("\nINFO: FW version %d.%d.%d was detected\n" , reading[10], reading[9], reading[8]);
}

uint8_t qmi8610_init(void)
{
	uint8_t fisimu_chip_id = 0x00;

	qmi8610_read_reg(Qmi8610Register_WhoAmI, &fisimu_chip_id, 1);
	qst_printf("Qmi8610Register_WhoAmI=0x%x \n", fisimu_chip_id);
	if(fisimu_chip_id == 0xfc)
	{
		fisimu_config.inputSelection = QMI8610_CONFIG_ACCGYR_ENABLE;
		fisimu_config.accRange = AccRange_8g;
		fisimu_config.accOdr = AccOdr_1024Hz;
		fisimu_config.gyrRange = GyrRange_1024dps;
		fisimu_config.gyrOdr = GyrOdr_1024Hz;
		fisimu_config.magOdr = MagOdr_32Hz;
		fisimu_config.magDev = MagDev_AK8963;
		fisimu_config.aeOdr = AeOdr_32Hz;
	
		//qmi8610_serial_num();

		//qmi8610_config_acc(AccRange_8g, AccOdr_128Hz, Lpf_Enable, Hpf_Disable);
		//qmi8610_config_gyro(GyrRange_1024dps, GyrOdr_256Hz, Lpf_Enable, Hpf_Disable);
		//qmi8610_write_reg(Qmi8610Register_Ctrl7, 0x03);
		qmi8610_Config_apply(&fisimu_config);
#if defined(QMI8610_USE_FIFO)
		fifo_level = qmi8610_configureFifo(Fifo_WatermarkHalf, FifoSize_64, FifoMode_Stream);
#endif
		if(1)
		{
			uint8_t read_data = 0x00;
			qmi8610_read_reg(Qmi8610Register_Ctrl1, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl1=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl2, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl2=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl3, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl3=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl4, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl4=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl5, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl5=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl6, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl6=0x%x \n", read_data);
			qmi8610_read_reg(Qmi8610Register_Ctrl7, &read_data, 1);
			qst_printf("Qmi8610Register_Ctrl7=0x%x \n", read_data);
		}
//		qmi8610_set_layout(2);
	}
	else
	{
		fisimu_chip_id = 0;
	}

	return fisimu_chip_id;
}
