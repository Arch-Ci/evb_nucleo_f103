#ifndef __QMC6308_H
#define __QMC6308_H
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "qst_sw_i2c.h"

/* vendor chip id*/
#define QMC6308_IIC_ADDR				0x2c		// 6308: (0x2c<<1) 6310: (0x1c<<1)
#define QMC6310_IIC_ADDR_U				0x1c		// 6310U
#define QMC6310_IIC_ADDR_N				0x3c		// 6310N

#define QMC6308_CHIP_ID_REG				0x00
/*data output register*/
#define QMC6308_DATA_OUT_X_LSB_REG		0x01
#define QMC6308_DATA_OUT_X_MSB_REG		0x02
#define QMC6308_DATA_OUT_Y_LSB_REG		0x03
#define QMC6308_DATA_OUT_Y_MSB_REG		0x04
#define QMC6308_DATA_OUT_Z_LSB_REG		0x05
#define QMC6308_DATA_OUT_Z_MSB_REG		0x06
/*Status registers */
#define QMC6308_STATUS_REG				0x09
/* configuration registers */
#define QMC6308_CTL_REG_ONE				0x0A  /* Contrl register one */
#define QMC6308_CTL_REG_TWO				0x0B  /* Contrl register two */

/* Magnetic Sensor Operating Mode MODE[1:0]*/
#define QMC6308_SUSPEND_MODE			0x00
#define QMC6308_NORMAL_MODE				0x01
#define QMC6308_SINGLE_MODE				0x02
#define QMC6308_CONTINUE_MODE			0x03

#define QMC6308_ODR_10HZ				(0x00<<2)
#define QMC6308_ODR_50HZ				(0x01<<2)
#define QMC6308_ODR_100HZ				(0x02<<2)
#define QMC6308_ODR_200HZ				(0x03<<2)
#define QMC6308_ODR_CONTINUE		(0x00<<2)

#define QMC6308_OSR1_8					(0x00<<4)
#define QMC6308_OSR1_4					(0x01<<4)
#define QMC6308_OSR1_2					(0x02<<4)
#define QMC6308_OSR1_1					(0x03<<4)

#define QMC6308_OSR2_8					(0x03<<6)
#define QMC6308_OSR2_4					(0x02<<6)
#define QMC6308_OSR2_2					(0x01<<6)
#define QMC6308_OSR2_1					(0x00<<6)

#define QMC6308_SET_RESET_ON 			0x00
#define QMC6308_SET_ON 					0x01
#define QMC6308_SET_RESET_OFF 			0x02

#define QMC6308_RNG_30G					(0x00<<2)
#define QMC6308_RNG_12G					(0x01<<2)
#define QMC6308_RNG_8G					(0x02<<2)
#define QMC6308_RNG_2G					(0x03<<2)

#define QMC6308_ABS(X) 					((X) < 0.0f ? (-1 * (X)) : (X))
#define QMC6308_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))
//#define QMC6308_FILTER_SUPPORT
#define QMC6308_MODE_SWITCH

enum{
	QMC_NONE = 0,
	QMC_6308,
	QMC_6308_MPW,
	QMC_6310,

	QMC_TOTAL
};

enum{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,

	AXIS_TOTAL
};

typedef struct{
	signed char sign[3];
	unsigned char map[3];
} qmc6308_map;


typedef struct
{
	char			mode;
	unsigned short	count;
} qmc6308_set_ctl_t;


#define AVG_FILTER_SIZE		4
typedef struct
{
	char	init;
	int		array[AVG_FILTER_SIZE];
	//float	avg;
} QmcAvgFilter_t;


typedef struct
{
	unsigned char slave_addr;
	unsigned char chip_id;
	unsigned char chip_type;
	unsigned char op_mode;
#if defined(QMC6308_FILTER_SUPPORT)
	QmcAvgFilter_t avg[3];
#endif
#if defined(QMC6308_MODE_SWITCH)
	qmc6308_set_ctl_t	set_ctl;
#endif
}qmc6308_data_t;

int qmc6308_init(void);
int qmc6308_enable(void);
int qmc6308_disable(void);
void qmc6308_soft_reset(void);
int qmc6308_read_mag_xyz(float *data);
int qmc6308_do_selftest(short *data);
int qmc6308_odr_mode(unsigned char odr, unsigned char mode);
int qmc6308_range_sr(unsigned char range, unsigned char set_reset);

#endif

