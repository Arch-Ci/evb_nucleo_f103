#include <stdio.h>
#include <math.h>
#include "qst_algo_imu.h"

#define TAG "QstAlgo-Data"
//#define QST_IMU_DATA_RMS

typedef struct 
{
	short 						sign[3];
	unsigned short 		map[3];
}qst_layout;

#define MAX_CALI_COUNT      50
static qfloat offset_acc[3] = {0.0, 0.0, 0.0};
static qfloat offset_gyro[3] = {0.0, 0.0, 0.0};
static sensor_data_t acc_cali_min,acc_cali_max;
static qint  cali_count = 0;
#if defined(QST_IMU_DATA_RMS)
static sensor_data_t accel_data[MAX_CALI_COUNT];
static sensor_data_t gyro_data[MAX_CALI_COUNT];
static qfloat gyro_x_thres = 0.0f;
static qfloat gyro_y_thres = 0.0f;
static qfloat gyro_z_thres = 0.0f;
#endif

static QST_Filter gyro_filter_parameter = {0.369527459, 0.195815772, 0.391335815, 0.78267163, 0.391335815};
static QST_Filter accel_filter_parameter = {-0.747789085, 0.272214949, 0.131106466, 0.262212932, 0.131106466};
static QST_Filter_Buffer gyro_buffer[3];
static QST_Filter_Buffer accel_buffer[3];
//static QST_Filter_Lpf acc_lpf;

const qst_layout qst_coordinate[] = 
{
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1,-1, 1}, {0, 1, 2} },
	{ { 1,-1, 1}, {1, 0, 2} },

	{ {-1, 1, -1}, {0, 1, 2} },
	{ { 1, 1, -1}, {1, 0, 2} },
	{ { 1,-1, -1}, {0, 1, 2} },
	{ {-1,-1, -1}, {1, 0, 2} }
};

static matrix3	corr_mat =
{
	0.980085, 0.020402, 0.001114,
	0.020402, 0.978552, -0.003964,
	0.001114, -0.003964, 0.985341
};

static qst_layout imu_map;
static qst_layout mag_map;

qvoid qst_algo_imu_get_offset(qfloat bias_acc[3], qfloat bias_gyro[3])
{
	if(bias_acc)
	{
	    bias_acc[0] = offset_acc[0];
	    bias_acc[1] = offset_acc[1];
	    bias_acc[2] = offset_acc[2];
	}
	if(bias_gyro)
	{
	    bias_gyro[0] = offset_gyro[0];
	    bias_gyro[1] = offset_gyro[1];
	    bias_gyro[2] = offset_gyro[2];
	}
//	if(rms_gyro)
//	{
//	    rms_gyro[0] = gyro_x_thres;
//	    rms_gyro[1] = gyro_y_thres;
//	    rms_gyro[2] = gyro_z_thres;
//	}
}

qvoid qst_algo_imu_set_offset(qfloat bias_acc[3], qfloat bias_gyro[3])
{
	if(bias_acc)
	{
	    offset_acc[0] = bias_acc[0];
	    offset_acc[1] = bias_acc[0];
	    offset_acc[2] = bias_acc[0];
	}
	if(bias_gyro)
	{
	    offset_gyro[0] = bias_gyro[0];
	    offset_gyro[1] = bias_gyro[1];
	    offset_gyro[2] = bias_gyro[2];
	}
//	if(rms_gyro)
//	{
//	    gyro_x_thres = rms_gyro[0];
//	    gyro_y_thres = rms_gyro[1];
//	    gyro_z_thres = rms_gyro[2];
//	}
}

qint qst_algo_imu_cali(qfloat acc[3],qfloat gyro[3], qint side)
{
	LOGD("qst_algo_imu_cali: acc:%f  %f  %f gyro:%f  %f  %f\n", acc[0], acc[1], acc[2], gyro[0],gyro[1],gyro[2]);
    if(cali_count == 0)
    {
		offset_acc[0] = 0.0f;
		offset_acc[1] = 0.0f;
		offset_acc[2] = 0.0f;
		offset_gyro[0] = 0.0f;
		offset_gyro[1] = 0.0f;
		offset_gyro[2] = 0.0f;
		acc_cali_min.x = acc_cali_max.x = acc[0];
		acc_cali_min.y = acc_cali_max.y = acc[1];
		acc_cali_min.z = acc_cali_max.z = acc[2];
        cali_count++;
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
#if defined(QST_IMU_DATA_RMS)
        accel_data[cali_count].x =  acc[0];
        accel_data[cali_count].y =  acc[1];
        accel_data[cali_count].z =  acc[2];
        gyro_data[cali_count].x =  gyro[0];
        gyro_data[cali_count].y =  gyro[1];
        gyro_data[cali_count].z =  gyro[2];
#endif
        offset_acc[0] += acc[0];
        offset_acc[1] += acc[1];
        offset_acc[2] += acc[2];
        offset_gyro[0] += gyro[0];
        offset_gyro[1] += gyro[1];
        offset_gyro[2] += gyro[2];
		acc_cali_min.x = (acc[0]<acc_cali_min.x) ? acc[0] : acc_cali_min.x;
		acc_cali_max.x = (acc[0]>acc_cali_max.x) ? acc[0] : acc_cali_max.x;
		acc_cali_min.y = (acc[1]<acc_cali_min.y) ? acc[1] : acc_cali_min.y;
		acc_cali_max.y = (acc[1]>acc_cali_max.y) ? acc[1] : acc_cali_max.y;
		acc_cali_min.z = (acc[2]<acc_cali_min.z) ? acc[2] : acc_cali_min.z;
		acc_cali_max.z = (acc[2]>acc_cali_max.z) ? acc[2] : acc_cali_max.z;

        cali_count++;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
		qfloat avg_acc[3] = {0.0, 0.0, 0.0};
		qfloat avg_gyro[3] = {0.0, 0.0, 0.0};

		avg_acc[0] = offset_acc[0] / (MAX_CALI_COUNT-1);
		avg_acc[1] = offset_acc[1] / (MAX_CALI_COUNT-1);
		avg_acc[2] = offset_acc[2] / (MAX_CALI_COUNT-1);
		avg_gyro[0] = offset_gyro[0] / (MAX_CALI_COUNT-1);
		avg_gyro[1] = offset_gyro[1] / (MAX_CALI_COUNT-1);
		avg_gyro[2] = offset_gyro[2] / (MAX_CALI_COUNT-1);
		offset_acc[0] = offset_acc[1] = offset_acc[2] = 0.0f;
		offset_gyro[0] = offset_gyro[1] = offset_gyro[2] = 0.0f;
		LOGD("acc avg:%f %f %f gyr avg:%f %f %f\n", avg_acc[0],avg_acc[1],avg_acc[2],avg_gyro[0],avg_gyro[1],avg_gyro[2]);
		LOGD("acc min:%f %f %f max:%f %f %f\n", acc_cali_min.x,acc_cali_min.y,acc_cali_min.z,acc_cali_max.x,acc_cali_max.y,acc_cali_max.z);
// check data	
		if((QST_ALGO_ABS(acc_cali_max.x-acc_cali_min.x)>1)
			||(QST_ALGO_ABS(acc_cali_max.y-acc_cali_min.y)>1)
			||(QST_ALGO_ABS(acc_cali_max.z-acc_cali_min.z)>1))
		{
			cali_count = 0;
			LOGD("cali fail! device is not static! \n");
			return -1;
		}
		if((QST_ALGO_ABS(avg_acc[0])>3)
			||(QST_ALGO_ABS(avg_acc[1])>3)
			||(QST_ALGO_ABS(avg_acc[2]-9.807)>3))
		{
			cali_count = 0;
			LOGD("cali fail! Acc Offset too large! \n");
			return -1;
		}
		
		offset_acc[0] = (0.0f-avg_acc[0]);
        offset_acc[1] = (0.0f-avg_acc[1]);
		if(side)
        	offset_acc[2] = (9.807f-avg_acc[2]);
		else
        	offset_acc[2] = (-9.807f-avg_acc[2]);

        offset_gyro[0] = (0.0f-avg_gyro[0]);
        offset_gyro[1] = (0.0f-avg_gyro[1]);
        offset_gyro[2] = (0.0f-avg_gyro[2]);
		LOGD("cali offset: acc:%f %f %f gyro:%f %f %f\n", offset_acc[0], offset_acc[1], offset_acc[2], offset_gyro[0],offset_gyro[1],offset_gyro[2]);
// check data
#if defined(QST_IMU_DATA_RMS)
		{
			qint i = 0;
			qfloat rms_acc[3] = {0.0, 0.0, 0.0};
			qfloat rms_gyro[3] = {0.0, 0.0, 0.0};
			rms_acc[0] = rms_acc[1] = rms_acc[2] = 0.0f;
			rms_gyro[0] = rms_gyro[1] = rms_gyro[2] = 0.0f;
			
	        for(i = 1; i<MAX_CALI_COUNT; i++)
	        {
	            rms_acc[0] += (accel_data[i].x - avg_acc[0])*(accel_data[i].x - avg_acc[0]);
	            rms_acc[1] += (accel_data[i].y - avg_acc[1])*(accel_data[i].y - avg_acc[1]);
	            rms_acc[2] += (accel_data[i].z - avg_acc[2])*(accel_data[i].z - avg_acc[2]);

	            rms_gyro[0] += (gyro_data[i].x - avg_gyro[0])*(gyro_data[i].x - avg_gyro[0]);
	            rms_gyro[1] += (gyro_data[i].y - avg_gyro[1])*(gyro_data[i].y - avg_gyro[1]);
	            rms_gyro[2] += (gyro_data[i].z - avg_gyro[2])*(gyro_data[i].z - avg_gyro[2]);
	        }
	        rms_acc[0] = sqrtf(rms_acc[0]/(MAX_CALI_COUNT-1));
	        rms_acc[1] = sqrtf(rms_acc[1]/(MAX_CALI_COUNT-1));
	        rms_acc[2] = sqrtf(rms_acc[2]/(MAX_CALI_COUNT-1));
	        rms_gyro[0] = sqrtf(rms_gyro[0]/(MAX_CALI_COUNT-1));
	        rms_gyro[1] = sqrtf(rms_gyro[1]/(MAX_CALI_COUNT-1));
	        rms_gyro[2] = sqrtf(rms_gyro[2]/(MAX_CALI_COUNT-1));
			LOGD("cali rms: acc:%f %f %f gyro:%f %f %f\n", rms_acc[0], rms_acc[1], rms_acc[2], rms_gyro[0],rms_gyro[1],rms_gyro[2]);

	        gyro_x_thres = 6*rms_gyro[0];
	        gyro_y_thres = 6*rms_gyro[1];
	        gyro_z_thres = 6*rms_gyro[2];
		}
#endif
        cali_count = 0;
        return 1;
    }

    return 0;
}


qvoid qst_algo_imu_filter_init(qfloat schedule_dt)
{
#if 0
	if(schedule_dt > 6)
	{
		gyro_filter_parameter.a1 = 0.369527459;
		gyro_filter_parameter.a2 = 0.195815772;
		gyro_filter_parameter.b0 = 0.391335815;
		gyro_filter_parameter.b1 = 0.78267163;
		gyro_filter_parameter.b2 = 0.391335815;
		accel_filter_parameter.a1 = -1.14298046;
		accel_filter_parameter.a2 = 0.412801653;
		accel_filter_parameter.b0 = 0.0674552768;
		accel_filter_parameter.b1 = 0.134910554;
		accel_filter_parameter.b2 = 0.0674552768;
	}
	else
	{	
		gyro_filter_parameter.a1 = -0.747789085;
		gyro_filter_parameter.a2 = 0.272214949;
		gyro_filter_parameter.b0 = 0.131106466;
		gyro_filter_parameter.b1 = 0.262212932;
		gyro_filter_parameter.b2 = 0.131106466;
		accel_filter_parameter.a1 = -1.56101811;
		accel_filter_parameter.a2 = 0.641351581;
		accel_filter_parameter.b0 = 0.020083366;
		accel_filter_parameter.b1 = 0.0401667319;
		accel_filter_parameter.b2 = 0.020083366;
	}
#else
	set_cutoff_frequency(200, 30, &gyro_filter_parameter);
	set_cutoff_frequency(200, 30, &accel_filter_parameter);
#endif
}

qint qst_algo_imu_data_process(qfloat acc[3],qfloat gyro[3])
{
// add offset
	qint axis = 0;
	qfloat acc_t[3], gyro_t[3];

	acc_t[0] = (acc[0]+offset_acc[0]);
	acc_t[1] = (acc[1]+offset_acc[1]);
	acc_t[2] = (acc[2]+offset_acc[2]);
	gyro_t[0] = (gyro[0]+offset_gyro[0]);
	gyro_t[1] = (gyro[1]+offset_gyro[1]);
	gyro_t[2] = (gyro[2]+offset_gyro[2]);
// add offset
// noise check
#if defined(QST_IMU_DATA_RMS)
	if((gyro_t[0]>-gyro_x_thres) && (gyro_t[0]<gyro_x_thres))
	{
		gyro_t[0] = 0.0f;
	}
	if((gyro_t[1]>-gyro_y_thres) && (gyro_t[1]<gyro_y_thres))
	{
		gyro_t[1] = 0.0f;
	}
	if((gyro_t[2]>-gyro_z_thres) && (gyro_t[2]<gyro_z_thres))
	{
		gyro_t[2] = 0.0f;
	}
#endif
// noise check

// coordinate convert
	acc[0] = (qfloat)imu_map.sign[0]*acc_t[imu_map.map[0]];
	acc[1] = (qfloat)imu_map.sign[1]*acc_t[imu_map.map[1]];
	acc[2] = (qfloat)imu_map.sign[2]*acc_t[imu_map.map[2]];

	gyro[0] = (qfloat)imu_map.sign[0]*gyro_t[imu_map.map[0]];
	gyro[1] = (qfloat)imu_map.sign[1]*gyro_t[imu_map.map[1]];
	gyro[2] = (qfloat)imu_map.sign[2]*gyro_t[imu_map.map[2]];
// coordinate convert

// data filter
	//EFK_filter(acc, 0);
	//EFK_filter(gyro, 1);
	//qst_algo_acc_lfp(acc, &acc_lpf);	// yangzhiqiang remove
	for(axis = 0; axis < 3; axis++)
	{
		acc[axis] = Filter_Apply(acc[axis], &accel_buffer[axis], &accel_filter_parameter);
		gyro[axis] = Filter_Apply(gyro[axis], &gyro_buffer[axis], &gyro_filter_parameter);
	}
// data filter
	return 1;
}

qint qst_algo_mag_data_process(qfloat mag[3])
{
	qfloat mag_t[3];
#if 0
	mag_t[0] = mag[0];
	mag_t[1] = mag[1];
	mag_t[2] = mag[2];
#else
	// soft iron
	mag_t[0] = mag[0]*corr_mat.e00 + mag[1]*corr_mat.e01 + mag[2]*corr_mat.e02;
	mag_t[1] = mag[0]*corr_mat.e10 + mag[1]*corr_mat.e11 + mag[2]*corr_mat.e12;
	mag_t[2] = mag[0]*corr_mat.e20 + mag[1]*corr_mat.e21 + mag[2]*corr_mat.e22;
	// soft iron
#endif
	mag[0] = (qfloat)mag_map.sign[0]*mag_t[mag_map.map[0]];
	mag[1] = (qfloat)mag_map.sign[1]*mag_t[mag_map.map[1]];
	mag[2] = (qfloat)mag_map.sign[2]*mag_t[mag_map.map[2]];
	
	return 1;
}

qvoid qst_algo_layout_init(qint layout1, qint layout2)
{
	if((layout1 > 7)||(layout2>7))
		return;

	imu_map.sign[0] = qst_coordinate[layout1].sign[0];
	imu_map.sign[1] = qst_coordinate[layout1].sign[1];
	imu_map.sign[2] = qst_coordinate[layout1].sign[2];
	imu_map.map[0] = qst_coordinate[layout1].map[0];
	imu_map.map[1] = qst_coordinate[layout1].map[1];
	imu_map.map[2] = qst_coordinate[layout1].map[2];

	mag_map.sign[0] = qst_coordinate[layout2].sign[0];
	mag_map.sign[1] = qst_coordinate[layout2].sign[1];
	mag_map.sign[2] = qst_coordinate[layout2].sign[2];
	mag_map.map[0] = qst_coordinate[layout2].map[0];
	mag_map.map[1] = qst_coordinate[layout2].map[1];
	mag_map.map[2] = qst_coordinate[layout2].map[2];
}

