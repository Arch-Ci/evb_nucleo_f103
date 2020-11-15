
#include "stm32f10x_user.h"
#include <math.h>

//#define QST_EVB_DEMO_ACC
//#define QST_EVB_DEMO_MAG
//#define QST_EVB_DEMO_MAG_CALI
//#define QST_EVB_DEMO_PRESS		3
//#define QST_EVB_DEMO_TEMPEARTURE
#define QST_EVB_DEMO_9AXIS
//#define QST_EVB_DEMO_9AXIS_ALGO

//#define QST_EVB_DEMO_SLEEP_ENABLE

#define QST_PRINTF					printf

static qst_evb_t g_evb;

static qmp6988_data qmp6988;
#if (QST_EVB_DEMO_PRESS >= 2)
static qmp6988_data qmp6988_2;
#endif
#if (QST_EVB_DEMO_PRESS >= 3)
static qmp6988_data qmp6988_3;
#endif
#if defined(QST_EVB_DEMO_9AXIS)
static qst_fusion_data algo_imu;
#endif


#if defined(QST_EVB_DEMO_ACC)
extern uint32_t qst_step_algo(float norm, uint16_t ms);
#endif


#if defined(QST_EVB_DEMO_9AXIS)
static int qst_evb_imu_sw_cali(void);
void qst_evb_mag_cali(void);
void qst_evb_imu_send(void);
void qst_algo_imu_run(float dt);
#endif

void qst_delay(unsigned int delay)
{
	delay_ms(delay);
}

void bsp_led_set(uint8_t flag)
{
	if(flag)
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	else
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
}

void bsp_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);		// gyro reset pin

#if !defined(QST_USE_SPI)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);	// set i2c slave addr
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);		// cs high
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	//GPIO_ResetBits(GPIOC, GPIO_Pin_7);
}

// timer
void evb_setup_timer(TIM_TypeDef* timid, uint16_t ms, FunctionalState enable)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint32_t clock;
	uint8_t irq_channel;

	if(timid == TIM2)
	{
		clock = RCC_APB1Periph_TIM2;
		irq_channel = TIM2_IRQn;
	}
	else if(timid == TIM3)
	{
		clock = RCC_APB1Periph_TIM3;
		irq_channel = TIM3_IRQn;
	}
	else if(timid == TIM4)
	{
		clock = RCC_APB1Periph_TIM4;
		irq_channel = TIM4_IRQn;
	}
	// 5ms
	if(enable == ENABLE)
	{
		TIM_DeInit(timid);		
		RCC_APB1PeriphClockCmd(clock, ENABLE);
		if(ms <= 50)
		{
			TIM_TimeBaseStructure.TIM_Period = 1000*ms;//10600;//5300;//21200;//
			TIM_TimeBaseStructure.TIM_Prescaler = (64 - 1);
		}
		else if(ms < 500)
		{
			TIM_TimeBaseStructure.TIM_Period = 100*ms;//10600;//5300;//21200;//
			TIM_TimeBaseStructure.TIM_Prescaler = (640 - 1);		
		}
		else
		{
			TIM_TimeBaseStructure.TIM_Period = 1*ms;//10600;//5300;//21200;//
			TIM_TimeBaseStructure.TIM_Prescaler = (64000 - 1);
		}
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(timid, &TIM_TimeBaseStructure);

		//�жϷ����ʼ��
		TIM_ClearFlag(timid, TIM_FLAG_Update);		
	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
		NVIC_InitStructure.NVIC_IRQChannel = irq_channel;  //TIM2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�2��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�0��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���  
		TIM_ITConfig(timid, TIM_IT_Update, ENABLE);//��������ж� ,����CC3IE�����ж� 
		TIM_Cmd(timid, ENABLE);
	}
	else
	{
		//TIM_DeInit(timid);
		//RCC_APB1PeriphClockCmd(clock, ENABLE);
		//TIM_Cmd(timid, DISABLE);
		//TIM_ClearFlag(timid, TIM_FLAG_Update);
		//TIM_ITConfig(timid, TIM_IT_Update, DISABLE);
		//TIM_DeInit(timid);
		//RCC_APB1PeriphClockCmd(clock, DISABLE);
		TIM_ClearFlag(timid, TIM_FLAG_Update);
		TIM_ClearITPendingBit(timid, TIM_IT_Update);
		TIM_Cmd(timid, DISABLE);
	}
}


void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM1 , TIM_IT_Update);		
		g_evb.tim1_flag = 1;
	}		 	
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
		g_evb.tim2_flag = 1;
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) 
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
		g_evb.tim3_flag = 1;
	}
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET ) 
	{
		TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
		g_evb.tim4_flag = 1;
	}
}

static void evb_tim_handle(void)
{
	if(g_evb.tim1_flag)
	{
		g_evb.tim1_flag = 0;
		if(g_evb.tim1_func)
		{
			g_evb.tim1_func();
		}
	}
	
	if(g_evb.tim2_flag)
	{
		g_evb.tim2_flag = 0;
		if(g_evb.tim2_func)
		{
			g_evb.tim2_func();
		}
	}
	
	if(g_evb.tim3_flag)
	{
		g_evb.tim3_flag = 0;
		if(g_evb.tim3_func)
		{
			g_evb.tim3_func();
		}
	}
	if(g_evb.tim4_flag)
	{
		g_evb.tim4_flag = 0;
		if(g_evb.tim4_func)
		{
			g_evb.tim4_func();
		}
	}
}

// timer

// irq
void EXTI9_5_IRQHandler(void)		// IMU int1:PC7  int2:PA9
{
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line7);
		  g_evb.irq1_flag = 1;
	}
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line9);
		  g_evb.irq2_flag = 1;
	}	
}

void EXTI4_IRQHandler(void)			// PB4-int1		PB10--int2
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line4);
		  g_evb.irq1_flag = 1;
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line10);
		  g_evb.irq2_flag = 1;
	}
}


static void evb_irq_handle(void)
{
	if(g_evb.irq1_flag)
	{
		g_evb.irq1_flag = 0;
		if(g_evb.irq1_func)
			g_evb.irq1_func();
	}
	if(g_evb.irq2_flag)
	{
		g_evb.irq2_flag = 0;
		if(g_evb.irq2_func)
			g_evb.irq2_func();
	}
}


static void evb_setup_irq(int int_type)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//��GPIO AFIO��ʱ��  
	if(int_type & QST_NUCLEOPB4_ACC_INT1)
	{
	    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//��GPIO AFIO��ʱ��  
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    EXTI_ClearITPendingBit(EXTI_Line4);  
	    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4); 
	    EXTI_InitStructure.EXTI_Line= EXTI_Line4;
	    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
	    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	    EXTI_Init(&EXTI_InitStructure);  

	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;  
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
	    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	    NVIC_Init(&NVIC_InitStructure); 
	}

	if(int_type & QST_NUCLEOPB10_ACC_INT2)
	{
	    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//��GPIO AFIO��ʱ��  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

		EXTI_ClearITPendingBit(EXTI_Line10);  
	    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10); 
	    EXTI_InitStructure.EXTI_Line= EXTI_Line10;
	    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
	    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	    EXTI_Init(&EXTI_InitStructure);  

	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
	    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	    NVIC_Init(&NVIC_InitStructure);
	}

// add by yangzhiqiang
	if(int_type & QST_NUCLEOPC7_IMU_INT1)
	{
	    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//��GPIO AFIO��ʱ��  
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
	    GPIO_Init(GPIOC, &GPIO_InitStructure); 

	    EXTI_ClearITPendingBit(EXTI_Line7);  
	    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7); 
	    EXTI_InitStructure.EXTI_Line= EXTI_Line7;
	    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
	    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	    EXTI_Init(&EXTI_InitStructure);  

	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;         
	    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	    NVIC_Init(&NVIC_InitStructure); 
	}
	if(int_type & QST_NUCLEOPA9_IMU_INT2)
	{
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//��GPIO AFIO��ʱ��  
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
	    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	  
	    EXTI_ClearITPendingBit(EXTI_Line9);  
	    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource9); 
	    EXTI_InitStructure.EXTI_Line= EXTI_Line9;
	    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
	    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	    EXTI_Init(&EXTI_InitStructure);  
	          
	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
	    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	    NVIC_Init(&NVIC_InitStructure);
	}
	
	g_evb.irq1_flag = 0;
	g_evb.irq2_flag = 0;
	g_evb.irq1_func = NULL;
	g_evb.irq2_func = NULL;
}


#if defined(QST_EVB_DEMO_SLEEP_ENABLE)
int qst_evb_sleep_check(float acc[3], int max_count)
{
	static float acc_sum_last = 0;
	static int static_count = 0;
	float acc_sum_tmp = QST_ABS(acc[0])+QST_ABS(acc[1])+QST_ABS(acc[2]);
	
	if(QST_ABS((acc_sum_tmp-acc_sum_last)) < 0.5)
	{
		static_count++;
		if(static_count > max_count)
		{
			static_count = 0;
			return 1;
		}
	}
	else
	{
		static_count = 0;
	}
	acc_sum_last = acc_sum_tmp;

	return 0;
}
#endif


typedef struct
{
    float x;
    float y;
    float z;
} imu_data_t;

typedef struct
{
	short cali_flag;
    float acc_bias[3];
    float gyr_bias[3];
    float mag_bias[3];
} imu_offset_t;

#define MAX_CALI_COUNT      51
static imu_offset_t imu_offset;
static imu_data_t acc_cali_min,acc_cali_max;
static int  cali_count = 0;

int qst_imu_cali(float acc[3],float gyro[3], int side)
{
	QST_PRINTF("qst_algo_imu_cali: acc:%f  %f  %f gyro:%f  %f  %f\n", acc[0], acc[1], acc[2], gyro[0],gyro[1],gyro[2]);
    if(cali_count == 0)
    {
#if defined(QST_IMU_CALI_STORE)
		flash_read((void*)&imu_offset, sizeof(imu_offset));
		if(imu_offset.cali_flag == 1)
		{		
			QST_PRINTF("cali offset read: acc:%f %f %f gyro:%f %f %f\n", imu_offset.acc_bias[0], imu_offset.acc_bias[1], imu_offset.acc_bias[2], 
																imu_offset.gyr_bias[0],imu_offset.gyr_bias[1],imu_offset.gyr_bias[2]);
			return 1;
		}
#endif
		imu_offset.acc_bias[0] = imu_offset.acc_bias[1] = imu_offset.acc_bias[2] = 0.0f;
		imu_offset.gyr_bias[0] = imu_offset.gyr_bias[1] = imu_offset.gyr_bias[2] = 0.0f;
		imu_offset.mag_bias[0] = imu_offset.mag_bias[1] = imu_offset.mag_bias[2] = 0.0f;
		acc_cali_min.x = acc_cali_max.x = acc[0];
		acc_cali_min.y = acc_cali_max.y = acc[1];
		acc_cali_min.z = acc_cali_max.z = acc[2];
        cali_count++;
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
        imu_offset.acc_bias[0] += acc[0];
        imu_offset.acc_bias[1] += acc[1];
        imu_offset.acc_bias[2] += acc[2];
        imu_offset.gyr_bias[0] += gyro[0];
        imu_offset.gyr_bias[1] += gyro[1];
        imu_offset.gyr_bias[2] += gyro[2];
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
		float avg_acc[3] = {0.0, 0.0, 0.0};
		float avg_gyro[3] = {0.0, 0.0, 0.0};

		avg_acc[0] = imu_offset.acc_bias[0] / (MAX_CALI_COUNT-1);
		avg_acc[1] = imu_offset.acc_bias[1] / (MAX_CALI_COUNT-1);
		avg_acc[2] = imu_offset.acc_bias[2] / (MAX_CALI_COUNT-1);
		avg_gyro[0] = imu_offset.gyr_bias[0] / (MAX_CALI_COUNT-1);
		avg_gyro[1] = imu_offset.gyr_bias[1] / (MAX_CALI_COUNT-1);
		avg_gyro[2] = imu_offset.gyr_bias[2] / (MAX_CALI_COUNT-1);
		imu_offset.acc_bias[0] = imu_offset.acc_bias[1] = imu_offset.acc_bias[2] = 0.0f;
		imu_offset.gyr_bias[0] = imu_offset.gyr_bias[1] = imu_offset.gyr_bias[2] = 0.0f;
		QST_PRINTF("acc avg:%f %f %f gyr avg:%f %f %f\n", avg_acc[0],avg_acc[1],avg_acc[2],avg_gyro[0],avg_gyro[1],avg_gyro[2]);
		QST_PRINTF("acc min:%f %f %f max:%f %f %f\n", acc_cali_min.x,acc_cali_min.y,acc_cali_min.z,acc_cali_max.x,acc_cali_max.y,acc_cali_max.z);
// check data	
		if((QST_ABS(acc_cali_max.x-acc_cali_min.x)>1)
			||(QST_ABS(acc_cali_max.y-acc_cali_min.y)>1)
			||(QST_ABS(acc_cali_max.z-acc_cali_min.z)>1))
		{
			cali_count = 0;
			QST_PRINTF("cali fail! device is not static! \n");
			return -1;
		}
		if(side)
		{
			if((QST_ABS(avg_acc[0])>3) ||(QST_ABS(avg_acc[1])>3)||(QST_ABS(avg_acc[2]-9.807)>3))
			{
				cali_count = 0;
				QST_PRINTF("cali fail! Acc Offset too large! \n");
				return -1;
			}
		}
		else
		{
			if((QST_ABS(avg_acc[0])>3) ||(QST_ABS(avg_acc[1])>3)||(QST_ABS(avg_acc[2]+9.807)>3))
			{
				cali_count = 0;
				QST_PRINTF("cali fail! Acc Offset too large! \n");
				return -1;
			}
		}
		
		imu_offset.acc_bias[0] = (0.0f-avg_acc[0]);
        imu_offset.acc_bias[1] = (0.0f-avg_acc[1]);
		if(side)
        	imu_offset.acc_bias[2] = (9.807f-avg_acc[2]);
		else
        	imu_offset.acc_bias[2] = (-9.807f-avg_acc[2]);

        imu_offset.gyr_bias[0] = (0.0f-avg_gyro[0]);
        imu_offset.gyr_bias[1] = (0.0f-avg_gyro[1]);
        imu_offset.gyr_bias[2] = (0.0f-avg_gyro[2]);
		QST_PRINTF("cali offset: acc:%f %f %f gyro:%f %f %f\n", imu_offset.acc_bias[0], imu_offset.acc_bias[1], imu_offset.acc_bias[2], 
															imu_offset.gyr_bias[0],imu_offset.gyr_bias[1],imu_offset.gyr_bias[2]);
        cali_count = 0;
		// write cali data
#if defined(QST_IMU_CALI_STORE)
		imu_offset.cali_flag = 1;
		flash_write_u16((void*)&imu_offset, sizeof(imu_offset));
#endif
		// write cali data
        return 1;
    }

    return 0;
}

void qst_imu_add_offset(float acc[3],float gyro[3])
{
	if(acc)
	{
		acc[0] = (acc[0]+imu_offset.acc_bias[0]);
		acc[1] = (acc[1]+imu_offset.acc_bias[1]);
		acc[2] = (acc[2]+imu_offset.acc_bias[2]);
	}
	if(gyro)
	{
		gyro[0] = (gyro[0]+imu_offset.gyr_bias[0]);
		gyro[1] = (gyro[1]+imu_offset.gyr_bias[1]);
		gyro[2] = (gyro[2]+imu_offset.gyr_bias[2]);
	}
}

// axis convert
#if 0
static void qst_axis_convert2(float *raw, float *out, int layout)
{
	if(layout >=4 && layout <= 7)
	{
		out[2] = -raw[2];
	}
	else
	{
		out[2] = raw[2];
	}

	if(layout%2)
	{
		out[0] = raw[1];
		out[1] = raw[0];
	}
	else
	{
		out[0] = raw[0];
		out[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		out[0] = -out[0];
	}	
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		out[1] = -out[1];
	}
}
#endif

static void qst_axis_convert(float data[3], int layout)
{
	float raw[3];

	raw[0] = data[0];
	raw[1] = data[1];
	//raw[2] = data[2];
	if(layout >=4 && layout <= 7)
	{
		data[2] = -data[2];
	}
	//else
	//{
	//	data[2] = raw[2];
	//}

	if(layout%2)
	{
		data[0] = raw[1];
		data[1] = raw[0];
	}
	else
	{
		data[0] = raw[0];
		data[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data[0] = -data[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data[1] = -data[1];
	}
}
// axis convert


// clock
static void SYSCLK_Config_WakeUp(void)
{
#if defined(SYSCLK_USE_HSI)
	RCC_DeInit(); /*������RCC�Ĵ�������Ϊȱʡֵ */ 
	RCC_HSICmd(ENABLE); 
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)== RESET);//�ȴ�HSI���� 
	RCC_HCLKConfig(RCC_SYSCLK_Div1);   /*����AHBʱ�ӣ�HCLK�� RCC_SYSCLK_Div1����AHBʱ�� = ϵͳʱ*/	
	RCC_PCLK2Config(RCC_HCLK_Div1);	 /* ���ø���AHBʱ�ӣ�PCLK2)RCC_HCLK_Div1����APB2ʱ�� = HCLK*/	 
	RCC_PCLK1Config(RCC_HCLK_Div2); /*���õ���AHBʱ�ӣ�PCLK1��RCC_HCLK_Div2����APB1ʱ�� = HCLK / 2*/ 	 
	FLASH_SetLatency(FLASH_Latency_2);	/*����FLASH�洢����ʱʱ��������FLASH_Latency_2	2��ʱ����*/   
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  /*ѡ��FLASHԤȡָ�����ģ,Ԥȡָ����ʹ��*/ 
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);/*����PLLʱ��Դ����Ƶϵ����Ƶ��Ϊ8/2*16=64Mhz*/	 
	RCC_PLLCmd(ENABLE);	 /*ʹ��PLL */ 
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) ; /*���ָ����RCC��־λ(PLL׼���ñ�־)�������*/    
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  /*����ϵͳʱ�ӣ�SYSCLK�� */	
	while(RCC_GetSYSCLKSource() != 0x08);	 /*0x08��PLL��Ϊϵͳʱ�� */	   
#else
	RCC_DeInit(); /*������RCC�Ĵ�������Ϊȱʡֵ */ 
	RCC_HSEConfig(RCC_HSE_ON);		/* ʹ�� HSE */
	RCC_WaitForHSEStartUp();
	//while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	//{
		/* �ȴ� HSE ׼������ */
	//}
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	RCC_PLLCmd(ENABLE);		/* ʹ�� PLL */ 
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
		/* �ȴ� PLL ׼������ */
	}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	/* ѡ��PLL��Ϊϵͳʱ��Դ */
	while (RCC_GetSYSCLKSource() != 0x08)
	{	
		/* �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ */
	}
#endif
}
// clock


static void evb_exe_rx_buf(void)
{
	unsigned char *rx_buf;
	unsigned short len;

	if(bsp_uart_get_rx_buf(&rx_buf, &len))
	{
		if((rx_buf[0]=='s')&&(rx_buf[1]=='l')&&(rx_buf[2]=='e')&&(rx_buf[3]=='e')&&(rx_buf[4]=='p'))
		{
#if defined(QST_EVB_DEMO_ACC)
			if(g_evb.accel == QST_ACCEL_QMAX981)
			{
				qmaX981_writereg(0x11, 0x00);
			}
			else if(g_evb.accel == QST_ACCEL_QMA6100)
			{
				qma6100_writereg(0x11, 0x00);
			}
			QST_PRINTF("accel sleep\n");
#endif
		}
		else if((rx_buf[0]=='w')&&(rx_buf[1]=='a')&&(rx_buf[2]=='k')&&(rx_buf[3]=='e'))
		{
#if defined(QST_EVB_DEMO_ACC)
			if(g_evb.accel == QST_ACCEL_QMAX981)
			{
				qmaX981_writereg(0x11, 0x80);
			}
			else if(g_evb.accel == QST_ACCEL_QMA6100)
			{
				qma6100_writereg(0x11, 0x80);
			}
			QST_PRINTF("accel wake\n");
#endif
		}		
		else if((rx_buf[0]=='r')&&(rx_buf[1]=='s')&&(rx_buf[2]=='t'))
		{
#if defined(QST_EVB_DEMO_ACC)
			if(g_evb.accel == QST_ACCEL_QMAX981)
			{
				qmaX981_init();
			}
			else if(g_evb.accel == QST_ACCEL_QMA6100)
			{
				qma6100_init();
			}
			QST_PRINTF("accel reset\n");	
#endif
		}
		bsp_uart_rx_reset();
	}
}


void qst_evb_acc_drdy(void)
{
#if defined(QMAX981_INT_LATCH)||defined(QMA6100_INT_LATCH)
	unsigned char databuf[4];
#endif

	if(g_evb.accel == QST_ACCEL_QMAX981)
	{
		qmaX981_read_xyz(g_evb.out.data);
#if defined(QMAX981_INT_LATCH)
		qmaX981_readreg(0x09, databuf, 4);
#endif
	}
	else if(g_evb.accel == QST_ACCEL_QMA6100)
	{
		qma6100_read_acc_xyz(g_evb.out.data);
#if defined(QMA6100_INT_LATCH)
		qma6100_readreg(0x09, databuf, 4);
#endif
	}	
	QMAX981_LOG("drdy	%f	%f	%f\n", g_evb.out.data[0], g_evb.out.data[1], g_evb.out.data[2]);
#if defined(QST_EVB_DEMO_SLEEP_ENABLE)
	if(qst_evb_sleep_check(g_evb.out.data, 500))
	{
		QST_PRINTF("acc enter sleep \n");
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
		SYSCLK_Config_WakeUp();
		QST_PRINTF("acc exit sleep \n");
	}
#endif
}

void qst_evb_acc_irq1(void)
{
	int ret = 0;
	unsigned char databuf[4];

	if(g_evb.accel == QST_ACCEL_QMAX981)
	{
		ret = qmaX981_readreg(0x09, databuf, 4);
		if(ret != QMAX981_SUCCESS)
		{
			return;
		}
		//QST_PRINTF("qma7981_irq_hdlr [0x%x 0x%x 0x%x 0x%x]\n", databuf[0],databuf[1],databuf[2],databuf[3]);
		if(databuf[0]&0x07)
		{
			QST_PRINTF("any motion!\n");
		}
		if(databuf[0]&0x80)
		{
			QST_PRINTF("no motion!\n");
		}
#if defined(QMAX981_STEP_INT)
		if(databuf[1]&0x08)
		{
			QST_PRINTF("step int!--%d\n", qmaX981_read_stepcounter());
		}
#endif
#if defined(QMAX981_SIGNIFICANT_STEP_INT)
		if(databuf[1]&0x40)
		{
			QST_PRINTF("significant step int!\n");
		}
#endif	
#if defined(QMAX981_FIFO_FUNC)
		if(databuf[2]&0x20)
		{
			QST_PRINTF("FIFO FULL\n");
			qmaX981_read_fifo(qmaX981_get_fifo());
			qmaX981_exe_fifo(qmaX981_get_fifo());
		}
		if(databuf[2]&0x40)
		{
			QST_PRINTF("FIFO WMK\n");
			qmaX981_read_fifo(qmaX981_get_fifo());
			qmaX981_exe_fifo(qmaX981_get_fifo());
		}
#endif
#if defined(QMAX981_HAND_RAISE_DOWN)
		if(databuf[1]&0x02)
		{
			QST_PRINTF("HAND RAISE\n");
		}
		if(databuf[1]&0x04)
		{
			QST_PRINTF("HAND DOWN\n");
		}
#endif
	}
	else if(g_evb.accel == QST_ACCEL_QMA6100)
	{
		ret = qma6100_readreg(QMA6100_INT_STAT0, databuf, 4);
		if(ret == QMA6100_FAIL)
		{
			QST_PRINTF("qma6100_irq_hdlr read status fail!\n");
			return;
		}
		if(databuf[2]&0x10)
		{	
			int raw[3];
			qma6100_read_raw_xyz(raw);
			QST_PRINTF("drdy	%d	%d	%d\n",raw[0],raw[1],raw[2]);
		}
#if defined(QMA6100_FIFO_FUNC)
		if((databuf[2]&0x40))
		{
			QST_PRINTF("FIFO WMK\n");
			qma6100_read_fifo(qma6100_get_fifo());
			qma6100_exe_fifo(qma6100_get_fifo());
		}
		else if((databuf[2]&0x20)) 
		{
			QST_PRINTF("FIFO FULL\n");
			qma6100_read_fifo(qma6100_get_fifo());
			qma6100_exe_fifo(qma6100_get_fifo());
		}
#endif
		if(databuf[0]&0x07)
		{
			QST_PRINTF("any motion!\n");
		}
		if(databuf[1]&0x01)
		{
			QST_PRINTF("significant motion!\n");
		}
		if(databuf[0]&0x80)
		{
			QST_PRINTF("no motion!\n");
		}
		if(databuf[1]&0x80)
		{
			QST_PRINTF("SINGLE tap int!\n");
		}
		if(databuf[1]&0x20)
		{
			QST_PRINTF("DOUBLE tap int!\n");
		}
		if(databuf[1]&0x10)
		{
			QST_PRINTF("TRIPLE tap int!\n");
		}	
		if(databuf[2]&0x01)
		{
			QST_PRINTF("QUARTER tap int!\n");
		}
		if(databuf[1]&0x02)
		{
			QST_PRINTF("hand raise!\n");
		}
		if(databuf[1]&0x04)
		{
			QST_PRINTF("hand down!\n");
		}
	}
}

void qst_evb_acc_tim2(void)
{
	if(g_evb.accel == QST_ACCEL_QMA6100)
	{
		qma6100_read_acc_xyz(g_evb.out.data);
#if defined(QMA6100_STEPCOUNTER)
		g_evb.step = qma6100_read_stepcounter();
#endif
	}
	else if(g_evb.accel == QST_ACCEL_QMAX981)
	{
		qmaX981_read_xyz(g_evb.out.data);
#if defined(QMAX981_STEPCOUNTER)
		g_evb.step = qmaX981_read_stepcounter();
#endif
	}
	qst_axis_convert(g_evb.out.data, 0);
	QST_PRINTF("Acc: %f	%f	%f	step:%d\r\n", g_evb.out.x, g_evb.out.y, g_evb.out.z, g_evb.step);
#if defined(LCM_SD1306_SUPPORT)
	sd1306_show_num(1, 1, g_evb.step, 6, FONT_SIZE16);
#endif
#if defined(QST_EVB_DEMO_SLEEP_ENABLE)
	if(qst_evb_sleep_check(g_evb.out.data, 50))
	{
		QST_PRINTF("acc enter sleep \n");
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
		//PWR_EnterSTANDBYMode();
		SYSCLK_Config_WakeUp();
		QST_PRINTF("acc exit sleep \n");
	}
#endif
}

void qst_evb_acc_algo_step(void)
{
#if 0
	float				norm;

	if(g_evb.accel == QST_ACCEL_QMA6100)
	{
		qma6100_read_acc_xyz(g_evb.out.data);
	}
	else if(g_evb.accel == QST_ACCEL_QMAX981)
	{
		qmaX981_read_xyz(g_evb.out.data);
	}
	QST_PRINTF("%f	%f	%f\r\n", g_evb.out.x, g_evb.out.y, g_evb.out.z);
	norm = sqrtf(g_evb.out.data[0]*g_evb.out.data[0]+g_evb.out.data[1]*g_evb.out.data[1]+g_evb.out.data[2]*g_evb.out.data[2]);
	g_evb.step = qst_step_algo(norm, 10);
#endif
}

#if defined(QST_EVB_DEMO_ACC)
void qst_evb_demo_acc(void)
{
	g_evb.out.sensor = QST_SENSOR_ACCEL;
	g_evb.report_mode = QST_REPORT_POLLING;

	evb_setup_irq(QST_NUCLEOPB4_ACC_INT1|QST_NUCLEOPC7_IMU_INT1);
	g_evb.irq1_func = qst_evb_acc_irq1;

	if(g_evb.report_mode == QST_REPORT_DRI)
	{
		g_evb.irq1_func = qst_evb_acc_drdy;		
	}
	else if(g_evb.report_mode == QST_REPORT_POLLING)
	{
		evb_setup_timer(TIM2, 100, ENABLE);
		g_evb.tim2_func = qst_evb_acc_tim2;
	}

	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}
#endif


#if 0
//#define FILTER_NUM	5
//#define FILTER_A	0.1f
#if 0
#define Factor	0.05f
#define R	0.00801f
#else
#define Factor	0.01f
#define R	0.000000414975f
#endif
#define Q	R*Factor
static float x_now	= 0.0f;
static float  x_mid	= 0.0f;
static float  p_mid	= 0.0f;
static float  x_last = 0.0f;
static float  p_last = 0.0f;
static float  p_now = 0.0f;
static float  kg = 0.0f;

static float qmp6988_press_filter(float* in)
{
	if(x_last < 1.0)
	{
		x_last = * in;
	}
    x_mid=x_last; 
    p_mid=p_last+Q; 
    kg=p_mid/(p_mid+R); 
    x_now=x_mid+kg*(*in -x_mid); 
    p_now=(1-kg)*p_mid;
    p_last = p_now; 
    x_last = x_now; 

    return x_now;
}
#endif

float qmp6988_calc_altitude(float press, float tempearture)
{
	float altitude;

//	press = qmp6988_press_filter(&press);
	altitude = (pow((101325/press),1/5.257)-1)*(tempearture+273.15)/0.0065;

	return altitude;
}

#if defined(QST_EVB_DEMO_PRESS)
void qst_evb_press_tim(void)
{
#if (QST_EVB_DEMO_PRESS > 1)
	g_evb.out.timestamp += 300;
	qmp6988_calc_pressure(&qmp6988, &g_evb.out.x, &g_evb.out.y);
	#if (QST_EVB_DEMO_PRESS >= 2)
	qmp6988_calc_pressure(&qmp6988_2, &g_evb.out2.x, &g_evb.out2.y);
	#endif
	#if (QST_EVB_DEMO_PRESS >= 3)
	qmp6988_calc_pressure(&qmp6988_3, &g_evb.out3.x, &g_evb.out3.y);
	#endif
	
	QST_PRINTF("%d:%d	A:	%d	B:	%d	C:	%d\n",
			(int)(g_evb.out.timestamp/1000),(int)(g_evb.out.timestamp%1000),
			(int)g_evb.out.x, (int)g_evb.out2.x, (int)g_evb.out3.x);
#else
	qmp6988_calc_pressure(&qmp6988, &g_evb.out.data[0], &g_evb.out.data[1]);
	g_evb.out.data[2] = qmp6988_calc_altitude(g_evb.out.data[0], g_evb.out.data[1]);
	QST_PRINTF("press:%f	temp:%f	altiude:%f\r\n", g_evb.out.data[0], g_evb.out.data[1], g_evb.out.data[2]);
#endif
}

void qst_evb_demo_press(void)
{
	QST_PRINTF("qst_evb_demo_press \n");
	g_evb.out.sensor = QST_SENSOR_PRESS;
	g_evb.report_mode = QST_REPORT_POLLING;
	evb_setup_irq(QST_NUCLEOPC7_IMU_INT1);
	evb_setup_timer(TIM3, 300, ENABLE);
	g_evb.tim3_func = qst_evb_press_tim;

	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}
#endif

#if defined(QST_EVB_DEMO_TEMPEARTURE)
void qst_evb_tempearture_tim(void)
{
	qmt200_read_raw();
	//QST_PRINTF("qst_evb_tempearture_tim\n");
}

void qst_evb_demo_tempearture(void)
{
	QST_PRINTF("qst_evb_demo_tempearture \n");
	g_evb.out.sensor = QST_SENSOR_TEMPEARTURE;
	g_evb.report_mode = QST_REPORT_POLLING;
	evb_setup_irq(QST_NUCLEO_INT_NONE);
	evb_setup_timer(TIM3, 300, ENABLE);
	g_evb.tim3_func = qst_evb_tempearture_tim;

	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}
#endif


#if defined(QST_EVB_DEMO_MAG_CALI)
static ICAL_IN_T q_mag_in;
//static ICAL_OUT_T g_mag_cali_r = 
//{
//	{17.885950, -1.656370, -4.373461},
//	49.157761,
//	3
//};

void qst_evb_mag_cali_tim(void)
{
	static short mag_out_count = 0;

	if(g_evb.accel == QST_ACCEL_QMA6100)
		qma6100_read_acc_xyz(q_mag_in.acc);
	else if(g_evb.accel == QST_ACCEL_QMAX981)
		qmaX981_read_xyz(q_mag_in.acc);

	if(g_evb.mag == QST_MAG_QMC6308)
		qmc6308_read_mag_xyz(q_mag_in.mag);
	else if(g_evb.mag == QST_MAG_QMC7983)
		qmcX983_read_mag_xyz(q_mag_in.mag);

	qst_imu_add_offset(q_mag_in.acc, NULL);
	qst_axis_convert(q_mag_in.acc, 2);
	qst_axis_convert(q_mag_in.mag, 4);
	//qst_axis_convert(q_magcali.acc, 0);
	//qst_axis_convert(q_magcali.mag, 1);

	qst_mag_cali(&q_mag_in, 0);
	mag_out_count++;
	if(mag_out_count >= 20)
	{
		mag_out_count = 0;
		QST_PRINTF("\nyaw:%f(%d)	pitch:%f	roll:%f", q_mag_in.euler[0],qst_mag_get_accuracy(),q_mag_in.euler[1],q_mag_in.euler[2]);
	}
}
#endif

#if defined(QST_EVB_DEMO_MAG)
void qst_evb_mag_tim2(void)
{
	if(g_evb.mag == QST_MAG_QMC6308)
		qmc6308_read_mag_xyz(g_evb.out.data);
	else if(g_evb.mag == QST_MAG_QMC7983)
		qmcX983_read_mag_xyz(g_evb.out.data);
	QST_PRINTF("mag:%f	%f	%f\n",g_evb.out.data[0],g_evb.out.data[1],g_evb.out.data[2]);
}

void qst_evb_demo_mag(void)
{
#if defined(QST_EVB_DEMO_MAG_CALI)
	int imu_cali = 0;
#endif
	g_evb.out.sensor = QST_SENSOR_MAG;
	g_evb.report_mode = QST_REPORT_POLLING;
	qst_delay(100);
#if defined(QST_EVB_DEMO_MAG_CALI)
	while(imu_cali == 0)
	{
		if(g_evb.accel == QST_ACCEL_QMA6100)
			qma6100_read_acc_xyz(q_mag_in.acc);
		else if(g_evb.accel == QST_ACCEL_QMAX981)
			qmaX981_read_xyz(q_mag_in.acc);

		imu_cali = qst_imu_cali(q_mag_in.acc, q_mag_in.acc, 1);
		qst_delay(20);
	}
	//	ical_Init(0, &g_mag_cali_r);
	g_evb.tim2_func = qst_evb_mag_cali_tim;
#else
	g_evb.tim2_func = qst_evb_mag_tim2;
#endif

	evb_setup_irq(QST_NUCLEO_INT_NONE);
	evb_setup_timer(TIM2, 20, ENABLE);

	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}
#endif

#if defined(QST_EVB_DEMO_9AXIS)

#if defined(QST_EVB_DEMO_9AXIS_ALGO)
void qst_algo_imu_init(void)
{
	if(g_evb.accgyro == QST_ACCGYRO_QMI8610)
	{
		qmi8610_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
	}
	else if(g_evb.accgyro == QST_ACCGYRO_QMI8658)
	{
		Qmi8658_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
	}
	qst_filter_init(5);
}
#endif

void qst_algo_imu_run(float dt)
{
#if defined(QST_EVB_DEMO_9AXIS_ALGO)
	static unsigned int imu_algo_count = 0;
#endif

	if(g_evb.accgyro == QST_ACCGYRO_QMI8610)
	{
		if(qmi8610_readStatus0()&0x33)
			qmi8610_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
		else
			return;
	}
	else if(g_evb.accgyro == QST_ACCGYRO_QMI8658)
	{
		if(Qmi8658_readStatus0()&0x33)
			Qmi8658_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
		else
			return;
	}
#if defined(QST_EVB_DEMO_9AXIS_ALGO)
	qst_imu_add_offset(algo_imu.acc, algo_imu.gyr);
	qst_axis_convert(algo_imu.acc, 7);
	qst_axis_convert(algo_imu.gyr, 7);
	qst_imu_fusion(dt, algo_imu.acc, algo_imu.gyr, algo_imu.mag, algo_imu.quat, algo_imu.euler);
	imu_algo_count++;
	if(imu_algo_count >= 4)
	{
		imu_algo_count = 0;
		qst_evb_mag_cali();		// mag cali
		qst_evb_imu_send();		// send msg
	}
#else
	qst_evb_imu_send();
#endif
}


static int qst_evb_imu_sw_cali(void)
{
	int imu_cali = 0;

#if defined(QST_EVB_DEMO_9AXIS_ALGO)
	if(g_evb.accgyro == QST_ACCGYRO_QMI8610)
	{
		if(qmi8610_readStatus0()&0x22)
			qmi8610_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
		else
			return 0;
	}
	else if(g_evb.accgyro == QST_ACCGYRO_QMI8658)
	{
		if(Qmi8658_readStatus0()&0x22)
			Qmi8658_read_xyz(algo_imu.acc, algo_imu.gyr, NULL);
		else
			return 0;
	}
	imu_cali = qst_imu_cali(algo_imu.acc, algo_imu.gyr, 1);
#else
	imu_cali = 1;
#endif

	return imu_cali;
}

// timer is 5 ms
void qst_evb_mag_cali(void)
{
#if defined(QST_EVB_DEMO_9AXIS_ALGO)

	float magDataIn[3];

	if(g_evb.mag == QST_MAG_QMC6308)
		qmc6308_read_mag_xyz(algo_imu.mag);
	else if(g_evb.mag == QST_MAG_QMC7983)
		qmcX983_read_mag_xyz(algo_imu.mag);
	else
		return;

#if 1
	magDataIn[0] = algo_imu.mag[0]*31.25;
	magDataIn[1] = algo_imu.mag[1]*31.25;
	magDataIn[2] = algo_imu.mag[2]*31.25;
	// cali mag
	process(magDataIn, HAL_GetTick());
	algo_imu.mag[0] = magDataIn[0]/31.25;//uT
	algo_imu.mag[1] = magDataIn[1]/31.25;
	algo_imu.mag[2] = magDataIn[2]/31.25;
	// mag data coordinate convert
#else
	ICAL_IN_T mag_in;

	mag_in.acc[0] = algo_imu.acc[0];
	mag_in.acc[1] = algo_imu.acc[1];
	mag_in.acc[2] = algo_imu.acc[2];
	mag_in.mag[0] = algo_imu.mag[0];
	mag_in.mag[1] = algo_imu.mag[1];
	mag_in.mag[2] = algo_imu.mag[2];
	qst_mag_cali(&mag_in, HAL_GetTick());
	algo_imu.mag[0] = mag_in.mag[0];
	algo_imu.mag[1] = mag_in.mag[1];
	algo_imu.mag[2] = mag_in.mag[2];
	#endif
#endif
	qst_axis_convert(algo_imu.mag, 5);
}


void qst_evb_imu_send(void)
{
#if defined(QST_EVB_DEMO_9AXIS_ALGO)
	#if defined(QST_IMU_ANO_TC)
	qst_send_imu_data(algo_imu.euler, algo_imu.gyr, algo_imu.acc, algo_imu.mag, 0);
	#else
	//qst_evb_plot_sws(algo_imu.acc, algo_imu.gyr);
	//QST_PRINTF("%d(%d)	%d	%d\n", (int)algo_imu.euler[0],qst_mag_get_accuracy(),(int)algo_imu.euler[1],(int)algo_imu.euler[2]);
	QST_PRINTF("%d(%d)	%d	%d\n", (int)algo_imu.euler[0],get_mag_accuracy(),(int)algo_imu.euler[1],(int)algo_imu.euler[2]);
	#endif
#else
	QST_PRINTF("%f	%f	%f	%f	%f	%f\n", algo_imu.acc[0],algo_imu.acc[1],algo_imu.acc[2],algo_imu.gyr[0],algo_imu.gyr[1],algo_imu.gyr[2]);
#endif
}

void qst_evb_imu_sleep(void)
{
	QST_PRINTF("qst_evb_imu_sleep \n");
	qmi8610_enableWakeOnMotion();	
	qst_delay(100);
	//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	PWR_EnterSTANDBYMode();
	SYSCLK_Config_WakeUp();
	QST_PRINTF("qst_evb_imu_sleep exit \n");
	qmi8610_disableWakeOnMotion();
}


void qst_evb_imu_irq1(void)
{
	QST_PRINTF("qst_evb_imu_irq1 \n");
}

void qst_evb_imu_irq2(void)
{
	qst_algo_imu_run(0.004);
#if defined(QST_EVB_DEMO_SLEEP_ENABLE)
	if(qst_evb_sleep_check(algo_imu.acc, 500))
	{
		qst_evb_imu_sleep();
	}
#endif
}

void qst_evb_imu_tim(void)
{
	qst_algo_imu_run(0.005);
#if defined(QST_EVB_DEMO_SLEEP_ENABLE)
	if(qst_evb_sleep_check(algo_imu.acc, 500))
	{
		qst_evb_imu_sleep();
	}
#endif
}

void qst_evb_demo_9axis(void)
{
	QST_PRINTF("qst_evb_demo_9axis \n");
	qst_delay(500);
	g_evb.out.sensor = QST_SENSOR_ACCEL;
	g_evb.out2.sensor = QST_SENSOR_GYRO;
	g_evb.report_mode = QST_REPORT_POLLING;
#if defined(QST_EVB_DEMO_9AXIS_ALGO)
	qst_algo_imu_init();
#endif
	while(qst_evb_imu_sw_cali() == 0)
	{
	}
	if(g_evb.report_mode == QST_REPORT_DRI)
	{
		evb_setup_irq(QST_NUCLEOPC7_IMU_INT1|QST_NUCLEOPA9_IMU_INT2);
		g_evb.irq1_func = NULL;	//qst_evb_imu_irq1;
		g_evb.irq2_func = qst_evb_imu_irq2;
	}	
	else if(g_evb.report_mode == QST_REPORT_POLLING)
	{
		evb_setup_irq(QST_NUCLEOPC7_IMU_INT1);
		evb_setup_timer(TIM2, 5, ENABLE);
		g_evb.tim2_func = qst_evb_imu_tim;
	}
	
	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}
#endif


void qst_evb_null_tim(void)
{
	QST_PRINTF("no sensor!\n");
}

void qst_evb_demo_null(void)
{
	evb_setup_irq(QST_NUCLEO_INT_NONE);
	evb_setup_timer(TIM2, 500, ENABLE);
	g_evb.tim2_func = qst_evb_null_tim;

	while(1)
	{
		evb_irq_handle();
		evb_tim_handle();
		evb_exe_rx_buf();
	}
}

int main(void)
{
	SYSCLK_Config_WakeUp();
	SysTick_Init(1);
	//SysTick_Enable(1);
	USART_Config();
	bsp_gpio_config();
	QST_PRINTF("QST demo start!\n");
	qst_delay(100);
#if defined(QST_USE_SW_I2C)
	i2c_sw_gpio_config();
	i2c_sw_gpio_config_2();
#else
	I2C_Bus_Init();
#endif
	
#if defined(QST_USE_SPI)
	//Spi_Init(0);
	Spi_Init(3);
#endif
	//RTC_NVIC_Config();
	//RTC_CheckAndConfig(&systmtime);

#if defined(LCM_SD1306_SUPPORT)
	sd1306_init();
#endif
	memset(&g_evb, 0, sizeof(g_evb));
#if defined(QST_EVB_DEMO_ACC)
	g_evb.init_sensor = QST_SENSOR_ACCEL;
#elif defined(QST_EVB_DEMO_PRESS)
	g_evb.init_sensor = QST_SENSOR_PRESS;
#elif defined(QST_EVB_DEMO_MAG)
	g_evb.init_sensor = QST_SENSOR_MAG;
#elif defined(QST_EVB_DEMO_9AXIS)
	g_evb.init_sensor = QST_SENSOR_MAG|QST_SENSOR_ACCGYRO;
#else
	g_evb.init_sensor = QST_SENSOR_TOTAL;
#endif

	if(g_evb.init_sensor & QST_SENSOR_ACCEL)
	{
		if(qma6100_init())
		{
			g_evb.accel = QST_ACCEL_QMA6100;
			QST_PRINTF("demo accel qma6100\n");
		}
		else if(qmaX981_init())
		{
			g_evb.accel = QST_ACCEL_QMAX981;
			QST_PRINTF("demo accel qmaX981\n");
		}
	}
	if(g_evb.init_sensor & QST_SENSOR_PRESS)
	{
#if (QST_EVB_DEMO_PRESS > 1)
		qmp6988.slave = 0x70;
		if(qmp6988_init(&qmp6988))
		{			
			QST_PRINTF("qmp6988-1 slave 0x%x find!\n",qmp6988.slave);
			g_evb.press = QST_PRESS_QMP6988;
		}
	#if (QST_EVB_DEMO_PRESS >= 2)
		qmp6988_2.slave = 0x56;
		if(qmp6988_init(&qmp6988_2))
		{
			QST_PRINTF("qmp6988-2 slave 0x%x find!\n", qmp6988_2.slave);
			g_evb.press = QST_PRESS_QMP6988;
		}
	#endif
	#if (QST_EVB_DEMO_PRESS >= 3)
		qmp6988_3.slave = 0x70;
		qmp6988_3.i2c_id = 1;
		if(qmp6988_init(&qmp6988_3))
		{
			QST_PRINTF("qmp6988-3 slave 0x%x find!\n" ,qmp6988_3.slave);
			g_evb.press = QST_PRESS_QMP6988;
		}
	#endif
#else
		qmp6988.slave = 0x70;
		if(qmp6988_init(&qmp6988))
		{
			g_evb.press = QST_PRESS_QMP6988;
		}
		else
		{
			qmp6988.slave = 0x56;
			if(qmp6988_init(&qmp6988))
			{
				g_evb.press = QST_PRESS_QMP6988;
			}
		}
#endif
	}
	if(g_evb.init_sensor & QST_SENSOR_MAG)
	{	
		if(qmc6308_init())
		{
			QST_PRINTF("mag qmc6308\n");
			g_evb.mag = QST_MAG_QMC6308;
		}
		else if(qmcX983_init())
		{
			QST_PRINTF("mag qmcX983\n");
			g_evb.mag = QST_MAG_QMC7983;
		}
	}
	if(g_evb.init_sensor & QST_SENSOR_ACCGYRO)
	{	
		QST_PRINTF("imu reset start\n");
		qst_delay(2000);
		QST_PRINTF("imu reset end\n");	
		if(qmi8610_init())
		{
			g_evb.accgyro = QST_ACCGYRO_QMI8610;
		}
		else if(Qmi8658_init())
		{
			g_evb.accgyro = QST_ACCGYRO_QMI8658;
		}

	}
#if defined(QST_EVB_DEMO_TEMPEARTURE)
	if(g_evb.init_sensor & QST_SENSOR_TEMPEARTURE)
	{
		if(qmt200_init())
		{
			qst_evb_demo_tempearture();
		}
	}
#endif
	
#if defined(QST_EVB_DEMO_ACC)
	if(g_evb.accel != QST_ACCEL_NONE)
		qst_evb_demo_acc();
#endif

#if defined(QST_EVB_DEMO_MAG)
	if(g_evb.mag != QST_MAG_NONE)
		qst_evb_demo_mag();
#endif

#if defined(QST_EVB_DEMO_PRESS)
	if(g_evb.press != QST_PRESS_NONE)
		qst_evb_demo_press();
#endif

#if defined(QST_EVB_DEMO_9AXIS)
	if(g_evb.accgyro != QST_ACCGYRO_NONE)
		qst_evb_demo_9axis();
#endif

	qst_evb_demo_null();
}


