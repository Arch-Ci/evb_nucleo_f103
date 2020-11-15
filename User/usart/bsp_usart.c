/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����õ�printf���ڣ��ض���printf������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 


#include "bsp_usart.h"

#define BSP_UART_RX_MAX		64

 typedef struct
 {
	 unsigned char	 buf[BSP_UART_RX_MAX];
	 unsigned short  index;
	 unsigned char	 finish;
 } bsp_usart_rx;
 
 static bsp_usart_rx rx_buf;

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE); 
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
	USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, DISABLE);
	//USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(DEBUG_USARTx, USART_FLAG_TC);

	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);

	rx_buf.finish = 0;
	rx_buf.index = 0;
}


///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}

void bsp_usart_send_str(unsigned char *str, int len)
{
	int i;
	for(i=0; i<len; i++)
	{	
		USART_SendData(DEBUG_USARTx, (uint8_t)str[i]);
		while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET); 	
	}
}

unsigned char bsp_uart_get_rx_buf(unsigned char **buf, unsigned short* len)
{
	if(rx_buf.finish)
	{
		*buf = rx_buf.buf;
		*len = rx_buf.index;
		return 1;
	}
	else
	{
		return 0;
	}
}

void bsp_uart_rx_reset(void)
{
	int i;

	rx_buf.finish = 0;
	rx_buf.index = 0;
	for(i=0; i<BSP_UART_RX_MAX; i++)
	{
		rx_buf.buf[i] = 0;
	}
}

void USART1_IRQHandler(void)
{
}


void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		if(rx_buf.finish == 0)
		{
			if(rx_buf.index == 0)
			{
				USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, ENABLE);
			}
			rx_buf.buf[rx_buf.index%128] = USART_ReceiveData(USART2);
			rx_buf.index++;
		}
	}
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		rx_buf.finish = 1;		
		USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);
	}
}

void USART3_IRQHandler(void)
{
}


