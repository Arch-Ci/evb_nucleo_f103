
#ifndef __STM32F10X_USER_H
#define __STM32F10X_USER_H

#ifdef __cplusplus
 extern "C" {
#endif 

#if defined(__CC_ARM)
#pragma anon_unions
#endif

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_spi.h"
#include "bsp_SysTick.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "qst_sw_i2c.h"
#include "bsp_spi.h"
#include "bsp_flash.h"
#include "sd1306.h"

#include "qmaX981.h"
#include "qma6100.h"
#include "qmcX983.h"
#include "qmp6988.h"
#include "qmi8610.h"
#include "qmi8658.h"
#include "qmc6308.h"
#include "px318j.h"

#include "qst_fusion.h"
//#include "../../algo/imu/qst_algo_imu.h"
#include "ICAL.h"
#include "qst_packet.h"


typedef void (*int_callback)(void);
enum 
{
	QST_REPORT_OFF,
	QST_REPORT_POLLING,
	QST_REPORT_DRI
};

typedef enum
{
	QST_SENSOR_NONE,
	QST_SENSOR_ACCEL,
	QST_SENSOR_MAG,
	QST_SENSOR_GYRO,
	QST_SENSOR_PRESS,
	QST_SENSOR_LIGHT,	
	QST_SENSOR_ACCGYRO,

	QST_SENSOR_TOTAL
} qst_sensor_type;

typedef enum
{
	QST_ACCEL_NONE,
	QST_ACCEL_QMAX981,
	QST_ACCEL_QMA6100,
	QST_ACCEL_QMI8610,
	QST_ACCEL_QMI8658,

	QST_ACCEL_TOTAL
} qst_acc_type;

typedef enum
{
	QST_MAG_NONE,
	QST_MAG_QMC7983,
	QST_MAG_QMC6308,
	QST_MAG_QMC6310,

	QST_MAG_TOTAL
} qst_mag_type;

typedef enum
{
	QST_GYRO_NONE,
	QST_GYRO_QMI8610,
	QST_GYRO_QMI8658,

	QST_GYRO_TOTAL
} qst_gyro_type;

typedef enum
{
	QST_ACCGYRO_NONE,
	QST_ACCGYRO_QMI8610,
	QST_ACCGYRO_QMI8658,

	QST_ACCGYRO_TOTAL
} qst_accgyro_type;

typedef enum
{
	QST_PRESS_NONE,
	QST_PRESS_QMP6988,

	QST_PRESS_TOTAL
} qst_press_type;

typedef struct
{
    union {
        int data[3];
        struct {
            int x;
            int y;
            int z;
        };
    };	
} sensors_raw_t;


typedef struct
{
	int sensor;
	float timestamp;
    union {
        float data[3];
        struct {
            float x;
            float y;
            float z;
        };
    };
} sensors_vec_t;

typedef struct
{
	float acc[3];
	float gyr[3];
	float mag[3];
	float quat[4];
	float euler[4];
} qst_algo_imu_t;

typedef struct
{
	qst_acc_type		accel;
	qst_mag_type		mag;
	qst_gyro_type		gyro;
	qst_press_type		press;
	qst_accgyro_type	accgyro;

	int					report_mode;

	unsigned char		irq1_flag;
	unsigned char		irq2_flag;
	int_callback		irq1_func;
	int_callback		irq2_func;

	int_callback		tim2_func;
	int_callback		tim3_func;
	int_callback		tim4_func;

	sensors_vec_t		out;
	sensors_vec_t		out2;
	unsigned int		step;
} qst_evb_t;



typedef struct
{
	float			acc_bis[3];
	float			gyro_bis[3];
	short			imu_cali;

	TRANSFORM_T		mag;
	short			mag_accuracy;
} qst_evb_offset_t;


#ifdef __cplusplus
}
#endif

#endif
