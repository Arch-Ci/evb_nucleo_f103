#ifndef QMI8658_H
#define QMI8658_H
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "qst_sw_i2c.h"
#include "bsp_spi.h"

#ifndef M_PI
#define M_PI	(3.14159265358979323846f)
#endif
#ifndef ONE_G
#define ONE_G	(9.807f)
#endif

#define QMI8658_CTRL7_DISABLE_ALL		(0x0)
#define QMI8658_CTRL7_ACC_ENABLE			(0x1)
#define QMI8658_CTRL7_GYR_ENABLE			(0x2)
#define QMI8658_CTRL7_MAG_ENABLE			(0x4)
#define QMI8658_CTRL7_AE_ENABLE			(0x8)
#define QMI8658_CTRL7_GYR_SNOOZE_ENABLE	(0x10)
#define QMI8658_CTRL7_ENABLE_MASK		(0xF)

#define QMI8658_CONFIG_ACC_ENABLE		QMI8658_CTRL7_ACC_ENABLE
#define QMI8658_CONFIG_GYR_ENABLE		QMI8658_CTRL7_GYR_ENABLE
#define QMI8658_CONFIG_MAG_ENABLE		QMI8658_CTRL7_MAG_ENABLE
#define QMI8658_CONFIG_AE_ENABLE			QMI8658_CTRL7_AE_ENABLE
#define QMI8658_CONFIG_ACCGYR_ENABLE		(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE)
#define QMI8658_CONFIG_ACCGYRMAG_ENABLE	(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_AEMAG_ENABLE		(QMI8658_CONFIG_AE_ENABLE | QMI8658_CONFIG_MAG_ENABLE)

#define QMI8658_STATUS1_CMD_DONE			(0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT		(0x04)

enum Qmi8658Register
{
	/*! \brief FIS device identifier register. */
	Qmi8658Register_WhoAmI = 0, // 0
	/*! \brief FIS hardware revision register. */
	Qmi8658Register_Revision, // 1
	/*! \brief General and power management modes. */
	Qmi8658Register_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	Qmi8658Register_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	Qmi8658Register_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	Qmi8658Register_Ctrl4, // 5
	/*! \brief Data processing settings. */
	Qmi8658Register_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	Qmi8658Register_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	Qmi8658Register_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	Qmi8658Register_Ctrl8, // 9
	/*! \brief Host command register. */
	Qmi8658Register_Ctrl9,	// 10
	/*! \brief Calibration register 1 most significant byte. */
	Qmi8658Register_Cal1_H = 11,
	/*! \brief Calibration register 1 least significant byte. */
	Qmi8658Register_Cal1_L,
	/*! \brief Calibration register 2 most significant byte. */
	Qmi8658Register_Cal2_H,
	/*! \brief Calibration register 2 least significant byte. */
	Qmi8658Register_Cal2_L,
	/*! \brief Calibration register 3 most significant byte. */
	Qmi8658Register_Cal3_H,
	/*! \brief Calibration register 3 least significant byte. */
	Qmi8658Register_Cal3_L,
	/*! \brief Calibration register 4 most significant byte. */
	Qmi8658Register_Cal4_H,
	/*! \brief Calibration register 4 least significant byte. */
	Qmi8658Register_Cal4_L,
	/*! \brief FIFO control register. */
	Qmi8658Register_FifoCtrl = 19,
	/*! \brief FIFO data register. */
	Qmi8658Register_FifoData,	// 20
	/*! \brief FIFO status register. */
	Qmi8658Register_FifoStatus,	// 21	
	/*! \brief Output data overrun and availability. */
	Qmi8658Register_StatusInt = 45,
	/*! \brief Output data overrun and availability. */
	Qmi8658Register_Status0,
	/*! \brief Miscellaneous status register. */
	Qmi8658Register_Status1,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_L,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_M,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_H,
	/*! \brief tempearture low. */
	Qmi8658Register_Tempearture_L,
	/*! \brief tempearture low. */
	Qmi8658Register_Tempearture_H,
	/*! \brief Accelerometer X axis least significant byte. */
	Qmi8658Register_Ax_L = 53,
	/*! \brief Accelerometer X axis most significant byte. */
	Qmi8658Register_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	Qmi8658Register_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	Qmi8658Register_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	Qmi8658Register_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	Qmi8658Register_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	Qmi8658Register_Gx_L = 59,
	/*! \brief Gyroscope X axis most significant byte. */
	Qmi8658Register_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	Qmi8658Register_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	Qmi8658Register_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	Qmi8658Register_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	Qmi8658Register_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	Qmi8658Register_Mx_L = 65,
	/*! \brief Magnetometer X axis most significant byte. */
	Qmi8658Register_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	Qmi8658Register_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	Qmi8658Register_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	Qmi8658Register_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	Qmi8658Register_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	Qmi8658Register_Q1_L = 73,
	/*! \brief Quaternion increment W most significant byte. */
	Qmi8658Register_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	Qmi8658Register_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	Qmi8658Register_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	Qmi8658Register_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	Qmi8658Register_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	Qmi8658Register_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	Qmi8658Register_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	Qmi8658Register_Dvx_L = 81,
	/*! \brief Velocity increment X most significant byte. */
	Qmi8658Register_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	Qmi8658Register_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	Qmi8658Register_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	Qmi8658Register_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	Qmi8658Register_Dvz_H,
	/*! \brief AttitudeEngine reg1. */
	Qmi8658Register_AeReg1 = 87,
	/*! \brief AttitudeEngine overflow flags. */
	Qmi8658Register_AeOverflow,

	Qmi8658Register_I2CM_STATUS = 110
};

enum Qmi8658_Ctrl9Command
{
	Ctrl9_Nop = 0,
	Ctrl9_GyroBias = 0x01,
	Ctrl9_ReqSdi = 0x03,
	Ctrl9_RetFifo = 0x04,
	Ctrl9_ReqFifo = 0x05,
	Ctrl9_I2cmWrite = 0x06,

	Ctrl9_ConfigureWakeOnMotion = 0x19,
	Ctrl9_SetAccelOffset = 100,
	Ctrl9_SetGyroOffset
};

enum Qmi8658_LpfConfig
{
	Lpf_Disable, /*!< \brief Disable low pass filter. */
	Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum Qmi8658_HpfConfig
{
	Hpf_Disable, /*!< \brief Disable high pass filter. */
	Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_StConfig
{
	St_Disable, /*!< \brief Disable high pass filter. */
	St_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_LpfMode
{
	A_LSP_MODE_0 = 0x00<<1,
	A_LSP_MODE_1 = 0x01<<1,
	A_LSP_MODE_2 = 0x02<<1,
	A_LSP_MODE_3 = 0x03<<1,
	
	G_LSP_MODE_0 = 0x00<<5,
	G_LSP_MODE_1 = 0x01<<5,
	G_LSP_MODE_2 = 0x02<<5,
	G_LSP_MODE_3 = 0x03<<5
};

enum Qmi8658_AccRange
{
	AccRange_2g = 0x00 << 4, /*!< \brief +/- 2g range */
	AccRange_4g = 0x01 << 4, /*!< \brief +/- 4g range */
	AccRange_8g = 0x02 << 4, /*!< \brief +/- 8g range */
	AccRange_16g = 0x03 << 4 /*!< \brief +/- 16g range */
};


enum Qmi8658_AccOdr
{
	AccOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	AccOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	AccOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	AccOdr_1000Hz = 0x03,  /*!< \brief High resolution 1000Hz output rate. */
	AccOdr_500Hz = 0x04,  /*!< \brief High resolution 500Hz output rate. */
	AccOdr_250Hz = 0x05, /*!< \brief High resolution 250Hz output rate. */
	AccOdr_125Hz = 0x06, /*!< \brief High resolution 125Hz output rate. */
	AccOdr_62_5Hz = 0x07, /*!< \brief High resolution 62.5Hz output rate. */
	AccOdr_31_25Hz = 0x08,  /*!< \brief High resolution 31.25Hz output rate. */
	AccOdr_LowPower_128Hz = 0x0c, /*!< \brief Low power 128Hz output rate. */
	AccOdr_LowPower_21Hz = 0x0d,  /*!< \brief Low power 21Hz output rate. */
	AccOdr_LowPower_11Hz = 0x0e,  /*!< \brief Low power 11Hz output rate. */
	AccOdr_LowPower_3Hz = 0x0f    /*!< \brief Low power 3Hz output rate. */
};

enum Qmi8658_GyrRange
{
	GyrRange_32dps = 0 << 4,   /*!< \brief +-32 degrees per second. */
	GyrRange_64dps = 1 << 4,   /*!< \brief +-64 degrees per second. */
	GyrRange_128dps = 2 << 4,  /*!< \brief +-128 degrees per second. */
	GyrRange_256dps = 3 << 4,  /*!< \brief +-256 degrees per second. */
	GyrRange_512dps = 4 << 4,  /*!< \brief +-512 degrees per second. */
	GyrRange_1024dps = 5 << 4, /*!< \brief +-1024 degrees per second. */
	GyrRange_2048dps = 6 << 4, /*!< \brief +-2048 degrees per second. */
	GyrRange_4096dps = 7 << 4  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum Qmi8658_GyrOdr
{
	GyrOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	GyrOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	GyrOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	GyrOdr_1000Hz = 0x03,	/*!< \brief High resolution 1000Hz output rate. */
	GyrOdr_500Hz	= 0x04,	/*!< \brief High resolution 500Hz output rate. */
	GyrOdr_250Hz	= 0x05,	/*!< \brief High resolution 250Hz output rate. */
	GyrOdr_125Hz	= 0x06,	/*!< \brief High resolution 125Hz output rate. */
	GyrOdr_62_5Hz 	= 0x07,	/*!< \brief High resolution 62.5Hz output rate. */
	GyrOdr_31_25Hz	= 0x08	/*!< \brief High resolution 31.25Hz output rate. */
};

enum Qmi8658_AeOdr
{
	AeOdr_1Hz = 0x00,  /*!< \brief 1Hz output rate. */
	AeOdr_2Hz = 0x01,  /*!< \brief 2Hz output rate. */
	AeOdr_4Hz = 0x02,  /*!< \brief 4Hz output rate. */
	AeOdr_8Hz = 0x03,  /*!< \brief 8Hz output rate. */
	AeOdr_16Hz = 0x04, /*!< \brief 16Hz output rate. */
	AeOdr_32Hz = 0x05, /*!< \brief 32Hz output rate. */
	AeOdr_64Hz = 0x06,  /*!< \brief 64Hz output rate. */
	AeOdr_128Hz = 0x07,  /*!< \brief 128Hz output rate. */
	/*!
	 * \brief Motion on demand mode.
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the Qmi8658_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	AeOdr_motionOnDemand = 128
};

enum Qmi8658_MagOdr
{
	MagOdr_1000Hz = 0x00,   /*!< \brief 1000Hz output rate. */
	MagOdr_500Hz = 0x01,	/*!< \brief 500Hz output rate. */
	MagOdr_250Hz = 0x02,	/*!< \brief 250Hz output rate. */
	MagOdr_125Hz = 0x03,	/*!< \brief 125Hz output rate. */
	MagOdr_62_5Hz = 0x04,	/*!< \brief 62.5Hz output rate. */
	MagOdr_31_25Hz = 0x05	/*!< \brief 31.25Hz output rate. */
};
	
enum Qmi8658_MagDev
{
	MagDev_AKM09918 = (0 << 3), /*!< \brief AKM09918. */
};

enum Qmi8658_AccUnit
{
	AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum Qmi8658_GyrUnit
{
	GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

struct FisImuConfig
{
	/*! \brief Sensor fusion input selection. */
	uint8_t inputSelection;
	/*! \brief Accelerometer dynamic range configuration. */
	enum Qmi8658_AccRange accRange;
	/*! \brief Accelerometer output rate. */
	enum Qmi8658_AccOdr accOdr;
	/*! \brief Gyroscope dynamic range configuration. */
	enum Qmi8658_GyrRange gyrRange;
	/*! \brief Gyroscope output rate. */
	enum Qmi8658_GyrOdr gyrOdr;
	/*! \brief AttitudeEngine output rate. */
	enum Qmi8658_AeOdr aeOdr;
	/*!
	 * \brief Magnetometer output data rate.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 * In this case the external magnetometer is sampled at the FIS output
	 * data rate, or at an integer divisor thereof such that the maximum
	 * sample rate is not exceeded.
	 */
	enum Qmi8658_MagOdr magOdr;

	/*!
	 * \brief Magnetometer device to use.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 */
	enum Qmi8658_MagDev magDev;
};


#define QMI8658_SAMPLE_SIZE (3 * sizeof(int16_t))
#define QMI8658_AE_SAMPLE_SIZE ((4+3+1) * sizeof(int16_t) + sizeof(uint8_t))
struct FisImuRawSample
{
	/*! \brief The sample counter of the sample. */
	uint8_t timestamp[3];
	/*!
	 * \brief Pointer to accelerometer data in the sample buffer.
	 *
	 * \c NULL if no accelerometer data is available in the buffer.
	 */
	uint8_t const* accelerometerData;
	/*!
	 * \brief Pointer to gyroscope data in the sample buffer.
	 *
	 * \c NULL if no gyroscope data is available in the buffer.
	 */
	uint8_t const* gyroscopeData;
	/*!
	 * \brief Pointer to magnetometer data in the sample buffer.
	 *
	 * \c NULL if no magnetometer data is available in the buffer.
	 */
	uint8_t const* magnetometerData;
	/*!
	 * \brief Pointer to AttitudeEngine data in the sample buffer.
	 *
	 * \c NULL if no AttitudeEngine data is available in the buffer.
	 */
	uint8_t const* attitudeEngineData;
	/*! \brief Raw sample buffer. */
	uint8_t sampleBuffer[QMI8658_SAMPLE_SIZE + QMI8658_AE_SAMPLE_SIZE];
	/*! \brief Contents of the FIS status 1 register. */
	uint8_t status1;
	//uint8_t status0;
	//uint32_t durT;
};

struct Qmi8658_offsetCalibration
{
	enum Qmi8658_AccUnit accUnit;
	float accOffset[3];
	enum Qmi8658_GyrUnit gyrUnit;
	float gyrOffset[3];
};

struct Qmi8658_sensitivityCalibration
{
	float accSensitivity[3];
	float gyrSensitivity[3];
};

enum Qmi8658_Interrupt
{
	/*! \brief FIS INT1 line. */
	Fis_Int1 = (0 << 6),
	/*! \brief FIS INT2 line. */
	Fis_Int2 = (1 << 6)
};

enum Qmi8658_InterruptInitialState
{
	InterruptInitialState_high = (1 << 7), /*!< Interrupt high. */
	InterruptInitialState_low  = (0 << 7)  /*!< Interrupt low. */
};

enum Qmi8658_WakeOnMotionThreshold
{
	WomThreshold_high = 128, /*!< High threshold - large motion needed to wake. */
	WomThreshold_low  = 32   /*!< Low threshold - small motion needed to wake. */
};


extern uint8_t Qmi8658_write_reg(uint8_t reg, uint8_t value);
extern uint8_t Qmi8658_read_reg(uint8_t reg, uint8_t* buf, uint16_t len);
extern uint8_t Qmi8658_init(void);
extern void Qmi8658_Config_apply(struct FisImuConfig const* config);
extern void Qmi8658_enableSensors(uint8_t enableFlags);
extern void Qmi8658_read_acc_xyz(float acc_xyz[3]);
extern void Qmi8658_read_gyro_xyz(float gyro_xyz[3]);
extern void Qmi8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count);
extern void Qmi8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count);
extern void Qmi8658_read_ae(float quat[4], float velocity[3]);
extern uint8_t Qmi8658_readStatus0(void);
extern uint8_t Qmi8658_readStatus1(void);
extern float Qmi8658_readTemp(void);
extern void Qmi8658_enableWakeOnMotion(void);
extern void Qmi8658_disableWakeOnMotion(void);
// for XKF3
extern void Qmi8658_processAccelerometerData(uint8_t const* rawData, float* calibratedData);
extern void Qmi8658_processGyroscopeData(uint8_t const* rawData, float* calibratedData);
extern void Qmi8658_read_rawsample(struct FisImuRawSample *sample);
extern void Qmi8658_applyOffsetCalibration(struct Qmi8658_offsetCalibration const* cal);
// for XKF3
extern uint16_t acc_lsb_div;
extern uint16_t gyro_lsb_div;
// fis210x
#endif
