

#ifndef GYRO_ALGO_H_
#define GYRO_ALGO_H_

#include <stdio.h>

typedef		void					qvoid; 	
typedef		char					qchar; 	
typedef		unsigned char			quchar; 
typedef		short					qshort; 	
typedef		unsigned short			qushort; 	
typedef		int						qint; 	
typedef		unsigned int			quint; 	
typedef		long long				qlong;
typedef		unsigned long long		qulog;
typedef		float					qfloat;
typedef		double					qdouble;

#define QST_ALGO_ABS(X) 				((X) < 0.0f ? (-1.0f * (X)) : (X))
#define QST_ALGO_PC_LOG
#if defined(QST_ALGO_ANDROID_LOG)
#include<android/log.h>
#define LOGD(...)         	__android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__)
#define LOGI(...)           __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__)
#define LOGW(...)         	__android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__)
#define LOGE(...)           __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__)
#define LOGF(...)           __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__)
#elif defined(QST_ALGO_PC_LOG)
#define LOGD				printf
#define LOGI		 		printf
#define LOGW				printf
#define LOGE				printf
#define LOGF				printf
#else
#define LOGD(...)		
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#define LOGF(...)
#endif

typedef enum
{
	QST_ALGO_VERSION_0,
	QST_ALGO_VERSION_1,
	QST_ALGO_VERSION_2,

	QST_ALGO_VERSION_END
}qst_algo_imu_v;

typedef struct
{
    qfloat x;
    qfloat y;
    qfloat z;
}sensor_data_t;


typedef struct
{
    qfloat q0;
    qfloat q1;
    qfloat q2;
    qfloat q3;
}quaternion_t;


typedef struct 		//欧拉（姿态）矩阵结构体
{
	qfloat T11,T12,T13,	 T21,T22,T23, T31,T32,T33;
}euler_martix;

typedef struct matrix3
{
	float e00, e01, e02, e10, e11, e12, e20, e21, e22;
} matrix3;

typedef struct
{

	float				X[4];
	qshort				YRP[3];
	sensor_data_t		buf_data[2];
}qst_imu_data_t;

typedef struct
{
	qchar	init;
	float	last_data[3];
}QST_Filter_Lpf;

typedef struct
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
}QST_Filter;

typedef struct
{
	float _delay_element_1;
	float _delay_element_2;
}QST_Filter_Buffer;

#ifdef __cplusplus
extern "C" {
#endif
// cali
qint qst_algo_imu_cali(qfloat acc[3],qfloat gyro[3], qint side);
qvoid qst_algo_imu_get_offset(qfloat bias_acc[3], qfloat bias_gyro[3]);
qvoid qst_algo_imu_set_offset(qfloat bias_acc[3], qfloat bias_gyro[3]);
// data process
qint qst_algo_imu_data_process(qfloat acc[3],qfloat gyro[3]);
qint qst_algo_mag_data_process(qfloat mag[3]);
qvoid qst_algo_layout_init(qint layout1, qint layout2);
qvoid qst_algo_imu_filter_init(qfloat schedule_dt);
// core algo
unsigned char qst_ahrs_update(float dt, float accel_ahrs[3], float gyro_ahrs[3], float mag_ahrs[3], float quaternion_output[4], float euler_output[3]);
//signed char qst_algo_motion_process(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z);

void set_cutoff_frequency(float sample_freq, float cutoff_freq, QST_Filter *filter);
float Filter_Apply(float sample, QST_Filter_Buffer *buffer, QST_Filter *filter);
qvoid qst_algo_acc_lfp(qfloat acc[3], QST_Filter_Lpf *acc_lpf); 


#ifdef __cplusplus
}
#endif

#endif /* GYRO_ALGO_H_ */
