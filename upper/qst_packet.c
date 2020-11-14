/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "./usart/bsp_usart.h"
#include "qst_packet.h"



int qst_putchar(int ch)
{
	USART_SendData(DEBUG_USARTx, (uint8_t) ch);
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET); 	

	return (ch);
}


void qst_send_str(unsigned char *str, int len)
{
	int i;
	for(i=0; i<len; i++)
	{	
		USART_SendData(DEBUG_USARTx, (uint8_t)str[i]);
		while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);	
	}
}


#if defined(QST_IMU_ANO_TC)
static unsigned			char data_to_send[50];
static qst_ano_tc_t		g_ano;

int qst_print_log (int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    int length, ii, i;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case QST_LOG_UNKNOWN:
    case QST_LOG_DEFAULT:
    case QST_LOG_VERBOSE:
    case QST_LOG_DEBUG:
    case QST_LOG_INFO:
    case QST_LOG_WARN:
    case QST_LOG_ERROR:
    case QST_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }

    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DEBUG;
    out[2] = priority;
    out[21] = '\r';
    out[22] = '\n';
		
		
    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
        
				for (i=0; i<PACKET_LENGTH; i++) {
          qst_putchar(out[i]);
        }
    }
    
            
    va_end(args);

    return 0;
}


void qst_send_data(unsigned char type, long *data)
{
    char out[PACKET_LENGTH];
    int i;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        break;
    default:
        return;
    }
    for (i=0; i<PACKET_LENGTH; i++) {
      qst_putchar(out[i]);
    }
}



void qst_send_imu_quat(long *quat)
{
    char out[PACKET_LENGTH];
    int i;
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (char)(quat[0] >> 24);
    out[4] = (char)(quat[0] >> 16);
    out[5] = (char)(quat[0] >> 8);
    out[6] = (char)quat[0];
    out[7] = (char)(quat[1] >> 24);
    out[8] = (char)(quat[1] >> 16);
    out[9] = (char)(quat[1] >> 8);
    out[10] = (char)quat[1];
    out[11] = (char)(quat[2] >> 24);
    out[12] = (char)(quat[2] >> 16);
    out[13] = (char)(quat[2] >> 8);
    out[14] = (char)quat[2];
    out[15] = (char)(quat[3] >> 24);
    out[16] = (char)(quat[3] >> 16);
    out[17] = (char)(quat[3] >> 8);
    out[18] = (char)quat[3];
    out[21] = '\r';
    out[22] = '\n';
		
    for (i=0; i<PACKET_LENGTH; i++) {
      qst_putchar(out[i]);
    }
}


void qst_send_imu_euler(float angle_pit, float angle_rol, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed)
{
	unsigned char i;
	unsigned char sum = 0;
	unsigned char _cnt=0;
	unsigned short _temp;
	int _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(angle_yaw > 180)
	{
		angle_yaw = angle_yaw-360;
	}
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		qst_putchar(data_to_send[i]);
}



void qst_send_imu_rawdata(short *Gyro,short *Accel, short *Mag)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
//	unsigned int _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE1(Accel[0]);
	data_to_send[_cnt++]=BYTE0(Accel[0]);
	data_to_send[_cnt++]=BYTE1(Accel[1]);
	data_to_send[_cnt++]=BYTE0(Accel[1]);
	data_to_send[_cnt++]=BYTE1(Accel[2]);
	data_to_send[_cnt++]=BYTE0(Accel[2]);
	
	data_to_send[_cnt++]=BYTE1(Gyro[0]);
	data_to_send[_cnt++]=BYTE0(Gyro[0]);
	data_to_send[_cnt++]=BYTE1(Gyro[1]);
	data_to_send[_cnt++]=BYTE0(Gyro[1]);
	data_to_send[_cnt++]=BYTE1(Gyro[2]);
	data_to_send[_cnt++]=BYTE0(Gyro[2]);
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Mag[0]);
	data_to_send[_cnt++]=BYTE0(Mag[0]);
	data_to_send[_cnt++]=BYTE1(Mag[1]);
	data_to_send[_cnt++]=BYTE0(Mag[1]);
	data_to_send[_cnt++]=BYTE1(Mag[2]);
	data_to_send[_cnt++]=BYTE0(Mag[2]);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		qst_putchar(data_to_send[i]);
}

void qst_send_imu_rawdata2(int alt_bar, unsigned short alt_csb)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;
	unsigned short _temp = 0;
	int _temp2 = 0;


	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;

	_temp2 = alt_bar;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	_temp = alt_csb;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;

	for(i=0;i<_cnt;i++)
		qst_putchar(data_to_send[i]);

}

void qst_send_imu_data(float *euler, float *Gyro, float *Accel, float *Mag, float Press)
{
	g_ano.acc_raw[0] = (short)(Accel[0]*1000.0f);
	g_ano.acc_raw[1] = (short)(Accel[1]*1000.0f);
	g_ano.acc_raw[2] = (short)(Accel[2]*1000.0f);
	g_ano.gyr_raw[0] = (short)(Gyro[0]*57.2957796f);
	g_ano.gyr_raw[1] = (short)(Gyro[1]*57.2957796f);
	g_ano.gyr_raw[2] = (short)(Gyro[2]*57.2957796f);
	g_ano.mag_raw[0] = (short)Mag[0];
	g_ano.mag_raw[1] = (short)Mag[1];
	g_ano.mag_raw[2] = (short)Mag[2];

	qst_send_imu_euler(euler[0], euler[1], euler[2], 0, 0x01, 1);
	qst_send_imu_rawdata(g_ano.gyr_raw, g_ano.acc_raw, g_ano.mag_raw);
	qst_send_imu_rawdata2((int)(Press*100), 0);
}


void qst_send_power(unsigned short votage, unsigned short current)
{
    unsigned char _cnt=0;
    unsigned short temp;
	unsigned char i=0;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;
    
    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    
    data_to_send[3] = _cnt-4;
    
    u8 sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;
    
	for(i=0;i<_cnt;i++)
		qst_putchar(data_to_send[i]);
}


void qst_send_version(unsigned char hardware_type, unsigned short hardware_ver,unsigned short software_ver,unsigned short protocol_ver,unsigned short bootloader_ver)
{
	unsigned char i=0;
	unsigned char _cnt=0;
	unsigned char sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
		qst_putchar(data_to_send[i]);
}
#endif

#if defined(QST_SSCOM_WAVEFORM)
void qst_evb_plotsscom(int x, int y, int z)
{
	int pktSize;
	plotsscom_t g_plotsim;

	g_plotsim.head = 0xCDAB;             //SimPlot packet header. Indicates start of data packet
	g_plotsim.len = 4*sizeof(plotsscom_t);      //Size of data in bytes. Does not include the header and size fields
	g_plotsim.buf[0] = (short)x*10;
	g_plotsim.buf[1] = (short)y*10;
	g_plotsim.buf[2] = (short)z*10;
	//g_plotsim.buf[3] = z;

	pktSize = sizeof(plotsscom_t); //Header bytes + size field bytes + data
	//IMPORTANT: Change to serial port that is connected to PC
	//Serial.write((uint8_t * )buffer, pktSize);	
	qst_send_str((unsigned char *)g_plotsim.buf, pktSize);
}
#endif

#if defined(QST_SWS_WAVEFORM)
unsigned char sws_head[2]={0x03,0xfc};//·¢ËÍÖ¡Í·
unsigned char sws_tail[2]={0xfc,0x03};//·¢ËÍÖ¡Î²

void qst_evb_plot_sws(float data1[3], float data2[3])
{
	unsigned char *p1,*p2,*p3;
	qst_send_str(sws_head, 2);
	p1 = (unsigned char *)&data1[0];
	p2 = (unsigned char *)&data1[1];
	p3 = (unsigned char *)&data1[2];
	printf("%c%c%c%c%c%c%c%c%c%c%c%c", p1[0],p1[1],p1[2],p1[3],
						p2[0],p2[1],p2[2],p2[3],
						p3[0],p3[1],p3[2],p3[3]);
	qst_send_str(sws_tail, 2);
}
#endif

