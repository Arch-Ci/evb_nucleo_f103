#ifndef QST_FUSION_H
#define QST_FUSION_H

//#define QST_ALGO_USE_FUSION

typedef struct
{
	float acc[3];
	float gyr[3];
	float mag[3];
	float euler[3];
	float quat[4];
	//float press;
    long long imu_tm;
    //long mag_tm;
} qst_fusion_data;

#ifdef __cplusplus
extern "C" {
#endif
void qst_filter_init(float schedule_dt);
unsigned char qst_imu_fusion(float dt, float accel_ahrs[3], float gyro_ahrs[3], float mag_ahrs[3], float quaternion_output[4], float euler_output[3]);
void qst_imu_get_euler(float *yaw, float *pitch, float *roll);
#ifdef __cplusplus
}
#endif

#endif
