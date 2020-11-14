/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup STM32L STM32L System Layer
 *  @brief  STM32L System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   packet.h
 *      @brief  Defines needed for sending data/debug packets via USB.
 */

#ifndef __QST_PACKET_H__
#define __QST_PACKET_H__

//#include "mltypes.h"
// 匿名上位机
//#define QST_IMU_ANO_TC
// sscom 波形图
//#define QST_SSCOM_WAVEFORM
//#define QST_SWS_WAVEFORM

typedef struct
{
	short acc_raw[3];
	short gyr_raw[3];
	short mag_raw[3];
} qst_ano_tc_t;

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} qst_packet_e;


#define QST_LOG_UNKNOWN		(0)
#define QST_LOG_DEFAULT		(1)
#define QST_LOG_VERBOSE		(2)
#define QST_LOG_DEBUG		(3)
#define QST_LOG_INFO		(4)
#define QST_LOG_WARN		(5)
#define QST_LOG_ERROR		(6)
#define QST_LOG_SILENT		(8)

#define BUF_SIZE					(256)
#define PACKET_LENGTH				(23)
#define PACKET_DEBUG				(1)
#define PACKET_QUAT					(2)
#define PACKET_DATA					(3)
#define min(a,b)					((a < b) ? a : b)
#define BYTE0(dwTemp)				(*(char *)(&dwTemp))
#define BYTE1(dwTemp)				(*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)				(*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)				(*((char *)(&dwTemp) + 3))

typedef struct plotsim
{
	short head;
	short len;
	short buf[3];
} plotsscom_t;

#ifdef __cplusplus
extern "C" {
#endif
#if defined(QST_IMU_ANO_TC)
void qst_send_str(unsigned char *str, int len);
void qst_send_data(unsigned char type, long *data);
void qst_send_imu_quat(long *quat);
void qst_send_imu_euler(float angle_pit, float angle_rol, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed);
void qst_send_imu_rawdata(short *Gyro,short *Accel, short *Mag);
void qst_send_imu_rawdata2(int alt_bar, unsigned short alt_csb);
void qst_send_imu_data(float *euler, float *Gyro, float *Accel, float *Mag, float Press);
void qst_send_power(unsigned short votage, unsigned short current);
void qst_send_version(unsigned char hardware_type, unsigned short hardware_ver,unsigned short software_ver,unsigned short protocol_ver,unsigned short bootloader_ver);
#endif
#if defined(QST_SSCOM_WAVEFORM)
void qst_evb_plotsscom(int x, int y, int z);
#endif
#if defined(QST_SWS_WAVEFORM)
void qst_evb_plot_sws(float data1[3], float data2[3]);
#endif
#ifdef __cplusplus
}
#endif


#endif /* __QST_PACKET_H__ */
