
#ifndef ICAL2_H_
#define ICAL2_H_

#define	ACCURACY_UNRELIABLE		0
#define	ACCURACY_LOW			1
#define	ACCURACY_MEDIUM			2
#define	ACCURACY_HIGH			3

#define ICAL_SUCCESS			1
#define ICAL_FAIL              	0

#define	ICAL_AXIS_X			0
#define	ICAL_AXIS_y			1
#define	ICAL_AXIS_Z			2
#define	ICAL_AXIS_NUM		3

typedef		char					INT8; 	
typedef		unsigned char			UINT8; 
typedef		short					INT16; 	
typedef		unsigned short			UINT16; 	
typedef		int						INT32; 	
typedef		unsigned int			UINT32; 	
typedef		long long				INT64;
typedef		unsigned long long		UINT64;
typedef		float					REAL;
typedef		double					DREAL;


typedef struct
{
	REAL acc[3];
	REAL mag[3];
	REAL euler[3];
} ICAL_IN_T;

typedef struct
{
	REAL offset[3];
	REAL rr;
	INT8 accuracy;
} ICAL_OUT_T;

#ifdef __cplusplus
extern "C" {
#endif
extern void ical_Init(long currentTimeMSec, ICAL_OUT_T* record);
extern void qst_mag_cali(ICAL_IN_T *data_in, long timestamp_ms);
extern INT8 qst_mag_get_accuracy(void);
extern void qst_mag_get_para(ICAL_OUT_T* record);

#ifdef __cplusplus
}
#endif

#endif /* ICAL_H_ */
