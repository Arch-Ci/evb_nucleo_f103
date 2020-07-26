
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "qst_fusion.h"
//#include "qmc_log.h"

#define TAG "QstAlgo-v16"

#define Pi 3.14159265358979f
#define Gravity 9.80665f
#define To_Degree	(180 / Pi)

typedef struct
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
} qst_butterworth_f;

typedef struct
{
	float _delay_element_1;
	float _delay_element_2;
} qst_butterworth_buf;


static float exInt = 0, eyInt = 0, ezInt = 0; 
//static float qst_kp = 0.5f, qst_ki = 0.1f;
//static float qst_kp = 2.0f, qst_ki = 0.005f;
//static float qst_kp = 1.6f, qst_ki = 0.001f;
//static float qst_kp = 10.0f, qst_ki = 0.008f;
#define qst_kp		(5.0f)
#define qst_ki		(0.005f)


static float qst_rot_mat[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
static float qst_q0 = 1.0f, qst_q1 = 0.0f, qst_q2 = 0.0f, qst_q3 = 0.0f;
static float qst_yaw = 0, qst_pitch = 0, qst_roll = 0;

// filter
static qst_butterworth_f qst_gyro_f = {0.369527459, 0.195815772, 0.391335815, 0.78267163, 0.391335815};
static qst_butterworth_f qst_accel_f = {-0.747789085, 0.272214949, 0.131106466, 0.262212932, 0.131106466};
static qst_butterworth_buf qst_gyro_buf[3];
static qst_butterworth_buf qst_accel_buf[3];


/*!
 * \brief set second order butterworth filtering Parameters.
* \input float sample_freq:accel and gyro filter sampling frequency.
* \input float cutoff_freq:accel and gyro filter cut-off frequency.
* \output Butterworth_Filter *filter:filter coefficient.
* \return void 
 */
static void qst_set_cutoff_frequency(float sample_freq, float cutoff_freq, qst_butterworth_f *filter)
{	
	float fr = sample_freq / cutoff_freq;
	float ohm = tanf(Pi / fr);
	float c = 1.0f + 2.0f * cosf(Pi / 4.0f) * ohm + ohm * ohm;
	filter->b0 = ohm * ohm / c;
	filter->b1 = 2.0f * filter->b0;
	filter->b2 = filter->b0;
	filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	filter->a2 = (1.0f - 2.0f * cosf(Pi / 4.0f) * ohm + ohm * ohm) / c;
}

void qst_filter_init(float schedule_dt)
{
	qst_set_cutoff_frequency(1000.0f/schedule_dt, 30, &qst_gyro_f);
	qst_set_cutoff_frequency(1000.0f/schedule_dt, 30, &qst_accel_f);
}

float qst_filter_apply(float sample, qst_butterworth_buf *buffer, qst_butterworth_f *filter) 
{
	//do the filtering
	float delay_element_0 = sample - buffer->_delay_element_1 * filter->a1 - buffer->_delay_element_2 * filter->a2;

	const float output = delay_element_0 * filter->b0 + buffer->_delay_element_1 * filter->b1 + buffer->_delay_element_2 * filter->b2;

	buffer->_delay_element_2 = buffer->_delay_element_1;
	buffer->_delay_element_1 = delay_element_0;

	//return the value. Should be no need to check limits
	return output;
}


/*
  wrap an angle defined in radians to -Pi ~ Pi (equivalent to +- 180 degrees)
 */
static float qst_wrap_pi(float angle_in_radians)
{
	while (angle_in_radians > Pi) 
	{
		angle_in_radians -= 2.0f * Pi;
	}
	while (angle_in_radians < -Pi) 
	{
		angle_in_radians += 2.0f * Pi;
	}
	return angle_in_radians;
}

/*!
 * \brief set second order butterworth filtering Parameters.
* \input float sample:accel and gyro filter input value.
* \input Butterworth_Buffer *buffer:accel and gyro filter buffer value.
* \input float cutoff_freq:accel and gyro filter cut-off frequency.
* \return filter output value
 */




// constrain a value
static float qst_constrain_float(float amt, float low, float high)
{
  return ((amt)<(low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}


static void qst_imuComputeRotationMatrix(void)
{
	float q2q2 = qst_q2 * qst_q2;
	float q2q3 = qst_q2 * qst_q3;
	float q1q1 = qst_q1 * qst_q1;
	float q1q2 = qst_q1 * qst_q2;
	float q1q3 = qst_q1 * qst_q3;
	float q0q1 = qst_q0 * qst_q1;
	float q0q2 = qst_q0 * qst_q2;
	float q0q3 = qst_q0 * qst_q3;
	float q3q3 = qst_q3 * qst_q3;

	qst_rot_mat[0][0] = 1 - 2 * (q2q2 + q3q3);
	qst_rot_mat[0][1] = 2 * (q1q2 - q0q3);
	qst_rot_mat[0][2] = 2 * (q1q3 + q0q2);

	qst_rot_mat[1][0] = 2 * (q1q2 + q0q3);
	qst_rot_mat[1][1] = 1 - 2 * (q1q1 + q3q3);
	qst_rot_mat[1][2] = 2 * (q2q3 - q0q1);

	qst_rot_mat[2][0] = 2 * (q1q3 - q0q2);
	qst_rot_mat[2][1] = 2 * (q2q3 + q0q1);
	qst_rot_mat[2][2] = 1-2 * (q1q1 + q2q2);
}


unsigned char qst_imu_fusion(float dt, float accel_ahrs[3], float gyro_ahrs[3], float mag_ahrs[3], float quaternion_output[4], float euler_output[3])
{
	float halfT = 0.5f * dt;
	
	float ex = 0.0f, ey = 0.0f, ez = 0.0f;
	float mx = 0.0f, my = 0.0f, mz = 0.0f;
	float ax = 0.0f, ay = 0.0f, az = 0.0f;
	float gx = 0.0f, gy = 0.0f, gz = 0.0f;
	float qa = 0.0f;
	float qb = 0.0f;
	float qc = 0.0f;
	float hx,hy,ez_ef;
	
	//Calculate general spin rate (rad/s)
	float spinRate = 0;
	float norm = 0;
	int axis = 0;

	for(axis=0; axis<3; axis++)
	{
		accel_ahrs[axis] = qst_filter_apply(accel_ahrs[axis], &qst_accel_buf[axis], &qst_accel_f);
		gyro_ahrs[axis] = qst_filter_apply(gyro_ahrs[axis], &qst_gyro_buf[axis], &qst_gyro_f);
	}
	mx = mag_ahrs[0];
	my = mag_ahrs[1];
	mz = mag_ahrs[2];

	//D("qst_imu_fusion gyro:%f %f %f	acc:%f %f %f\r\n", gyro_ahrs[0],gyro_ahrs[1],gyro_ahrs[2],accel_ahrs[0],accel_ahrs[1],accel_ahrs[2]);
	norm = sqrtf(mx * mx + my * my + mz * mz);
	//D("qst_imu_fusion mag_norm:%f \r\n", norm);
	if(norm > 0.0f) 
	{
		// Normalise magnetometer measurement
		mx = mx / norm;
		my = my / norm;
		mz = mz / norm;
		
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		hx = qst_rot_mat[0][0] * mx + qst_rot_mat[0][1] * my + qst_rot_mat[0][2] * mz;
		hy = qst_rot_mat[1][0] * mx + qst_rot_mat[1][1] * my + qst_rot_mat[1][2] * mz;
		ez_ef = -qst_wrap_pi(atan2f(hy, hx));
		
		ex = qst_rot_mat[2][0] * ez_ef;
		ey = qst_rot_mat[2][1] * ez_ef;
		ez = qst_rot_mat[2][2] * ez_ef;
	}

	ax = accel_ahrs[0];
	ay = accel_ahrs[1];
	az = accel_ahrs[2];
	
	norm = sqrtf(ax * ax + ay * ay + az * az);
	//D("qst_imu_fusion acc_norm:%f \r\n", norm);
	if(norm > Gravity * 0.9f && norm <  Gravity * 1.1f)
	{
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;
	
		ex += (-ay * qst_rot_mat[2][2] + az * qst_rot_mat[2][1]) * qst_kp;
		ey += (-az * qst_rot_mat[2][0] + ax * qst_rot_mat[2][2]) * qst_kp;
		ez += (-ax * qst_rot_mat[2][1] + ay * qst_rot_mat[2][0]) * qst_kp;
	}
	
	gx = gyro_ahrs[0];
	gy = gyro_ahrs[1];
	gz = gyro_ahrs[2];
	spinRate = sqrtf(gx * gx + gy * gy + gz * gz);
	//D("qst_imu_fusion spinRate:%f  dt:%f\r\n", spinRate, dt);
	if(spinRate < 1.075f) //10
	{
		exInt += ex * qst_ki * dt;
		eyInt += ey * qst_ki * dt;
		ezInt += ez * qst_ki * dt;
		exInt = qst_constrain_float(exInt, -0.05f, 0.05f);
		eyInt = qst_constrain_float(eyInt, -0.05f, 0.05f);
		ezInt = qst_constrain_float(ezInt, -0.05f, 0.05f);
	}
	else
	{
		exInt = 0.0f;
		eyInt = 0.0f;
		ezInt = 0.0f;	
	}
	
	gx = gx + ex + exInt;
	gy = gy + ey + eyInt;
	gz = gz + ez + ezInt;
	//LOGD("gyro_tmp:%f %f %f\r\n", gyro_tmp[0],gyro_tmp[1],gyro_tmp[2]);
	
	qa = qst_q0;
	qb = qst_q1;
	qc = qst_q2;
	
	//LOGD("halfT:%f\r\n", halfT);
	qst_q0 += (-qb * gx - qc * gy - qst_q3 * gz) * halfT;
	qst_q1 += (qa * gx + qc * gz - qst_q3 * gy) * halfT;
	qst_q2 += (qa * gy- qb * gz + qst_q3 * gx) * halfT;
	qst_q3 += (qa * gz + qb * gy - qc * gx) * halfT;
	
	norm = sqrtf(qst_q0 * qst_q0 + qst_q1 * qst_q1 + qst_q2 * qst_q2 + qst_q3 * qst_q3);
	qst_q0 = qst_q0 / norm;
	qst_q1 = qst_q1 / norm;
	qst_q2 = qst_q2 / norm;
	qst_q3 = qst_q3 / norm;
	//LOGD("qo:%f %f %f %f\r\n", q0,q1,q2,q3);
	quaternion_output[0] = qst_q0;
	quaternion_output[1] = qst_q1;
	quaternion_output[2] = qst_q2;
	quaternion_output[3] = qst_q3;
	
	qst_imuComputeRotationMatrix();

	qst_pitch = -asinf(qst_rot_mat[2][0]) * To_Degree;
	qst_roll = atan2f(qst_rot_mat[2][1], qst_rot_mat[2][2]) * To_Degree; 
	qst_yaw = atan2f(qst_rot_mat[1][0], qst_rot_mat[0][0]) * To_Degree; 

	if(qst_yaw < 0.0f)
	{
		qst_yaw += 360.0f;
	}
	euler_output[0] = qst_yaw;
	euler_output[1] = qst_pitch;
	euler_output[2] = qst_roll;
	//D("qst_imu_fusion out:%f %f %f", qst_yaw, qst_pitch, qst_roll);
	return 1;
}

void qst_imu_get_euler(float *yaw, float *pitch, float *roll)
{
	if(yaw)
		*yaw = qst_yaw;
	if(pitch)
		*pitch = qst_pitch;
	if(roll)
		*roll = qst_roll;	
}

