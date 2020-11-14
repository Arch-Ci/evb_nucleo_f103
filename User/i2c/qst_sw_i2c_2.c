
#include "qst_sw_i2c_2.h"


#define GPIO_PORT_I2C	GPIOA			/* GPIO端口 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOA		/* GPIO端口时钟 */

#define I2C_SCL_PIN		GPIO_Pin_5
#define I2C_SDA_PIN		GPIO_Pin_6

#define I2C_SCL_INPUT()			{GPIO_PORT_I2C->CRL&=0XFF0FFFFF;GPIO_PORT_I2C->CRL|=((uint32_t)8<<20);}
#define I2C_SCL_OUTPUT()		{GPIO_PORT_I2C->CRL&=0XFF0FFFFF;GPIO_PORT_I2C->CRL|=((uint32_t)3<<20);}
#define I2C_SDA_INPUT()			{GPIO_PORT_I2C->CRL&=0XF0FFFFFF;GPIO_PORT_I2C->CRL|=((uint32_t)8<<24);}	
#define I2C_SDA_OUTPUT() 		{GPIO_PORT_I2C->CRL&=0XF0FFFFFF;GPIO_PORT_I2C->CRL|=((uint32_t)3<<24);}	

#define I2C_SCL_1()				GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()				GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
#define I2C_SDA_1()				GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()				GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
#define I2C_SDA_READ()			((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)		/* 读SDA口线状态 */

static void i2c_Ack(void);
static void i2c_NAck(void);

static void i2c_delay_us(int nus)
{
	//volatile uint32_t i;

	while(nus--)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
#if 0
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP();

//		__NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP();
#endif
	}
}

#define i2c_Delay()		i2c_delay_us(2)

static void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();

	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
static uint8_t i2c_ReadByte(u8 ack)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		//I2C_SCL_1();
		//i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
static uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_SDA_OUTPUT();	//set data output
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	函 数 名: i2c_GPIO_Config
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_sw_gpio_config_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//GPIO_Mode_Out_OD; GPIO_Mode_Out_PP 	/* 开漏输出 */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}


uint8_t qst_sw_writereg_2(uint8_t slave, uint8_t reg_add,uint8_t reg_dat)
{
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
}

uint8_t qst_sw_writeregs_2(uint8_t slave, uint8_t reg_add, uint8_t *reg_dat, uint8_t len)
{
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	for(i=0; i<len; i++)
	{
		i2c_SendByte(reg_dat[i]);	
		if(i2c_WaitAck())
		{
			return 0;
		}
	}
	i2c_Stop();

	return 1;
}

uint8_t qst_sw_readreg_2(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num)
{
	//uint8_t ret;
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave|0x01);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}


