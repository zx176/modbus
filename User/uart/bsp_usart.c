/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �ض���c��printf������usart�˿�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��STM32 F103-�Ե� ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "bsp_usart.h"
/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =6;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
	
	
	// ���� DMA1 ͨ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

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
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	// �����ж����ȼ�����
	NVIC_Configuration();
	// ���� ���ڿ���IDEL �ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, ENABLE);  
  // ��������DMA����
	USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, ENABLE); 
	
	 // ��������DMA����
	USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Tx, ENABLE); 
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
}

//char Usart_Rx_Buf[USART_RBUFF_SIZE];
extern volatile unsigned char  ucRTUBuf[256];

void USARTx_DMA_Config()
{

	 
	  DMA_InitTypeDef DMA_RX_InitStructure, DMA_TX_InitStructure; // ���������ṹ�壬�ֱ�����RX��TX
		// ����DMAʱ��
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	
	
	// ���ý���DMAͨ�����Ӵ��ڵ��ڴ棩
    // ����ͨ����ʼ��
    DMA_RX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART_DR_ADDRESS; // �����ַ���������ݼĴ���
    DMA_RX_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ucRTUBuf;              // �ڴ��ַ�����ջ�������
    DMA_RX_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // �������赽�ڴ�
    DMA_RX_InitStructure.DMA_BufferSize = USART_RBUFF_SIZE;                    // ��������С
    DMA_RX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ������
    DMA_RX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ��ַ����
    DMA_RX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ�ȣ��ֽ�
    DMA_RX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ�ȣ��ֽ�
    DMA_RX_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // ѭ��ģʽ������ͨ����ѭ��ģʽ��
    DMA_RX_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ����ǳ���
    DMA_RX_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // ��ֹ�ڴ浽�ڴ�
    DMA_Init(USART_RX_DMA_CHANNEL, &DMA_RX_InitStructure);                     // ��ʼ������DMAͨ��

    // ���÷���DMAͨ�������ڴ浽���ڣ�
    DMA_TX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART_DR_ADDRESS; // �����ַ���������ݼĴ���
    DMA_TX_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;                     // �ڴ��ַ�����ͻ�����������ʱ����Ϊ0��ʵ�ʷ���ǰ��Ҫ����
    DMA_TX_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // �����ڴ浽����
    DMA_TX_InitStructure.DMA_BufferSize = 0;                                   // ��������������ʼΪ0������ǰ����
    DMA_TX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ������
    DMA_TX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ��ַ����
    DMA_TX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ�ȣ��ֽ�
    DMA_TX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ�ȣ��ֽ�
    DMA_TX_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ����ģʽ���������һ�κ�ֹͣ��
    DMA_TX_InitStructure.DMA_Priority = DMA_Priority_High;                     // ���ȼ�����
    DMA_TX_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // ��ֹ�ڴ浽�ڴ�
    DMA_Init(USART_TX_DMA_CHANNEL, &DMA_TX_InitStructure);                     // ��ʼ������DMAͨ��

    // ���DMA��־��������Ҫע�⣬���պͷ���ͨ���ı�־��Ҫ�������������ֻ�����ͨ��5����Ҫ����ʵ��ͨ��������
    // �������ͨ����ͨ��5������ͨ������һ��������ͨ��4��6������Ҫ�ֱ����
    // ��ͨ��5�����գ���ͨ��4�����ͣ�Ϊ����
    DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5); // �������ͨ����־
    DMA_ClearFlag(DMA1_FLAG_TC4 | DMA1_FLAG_HT4 | DMA1_FLAG_GL4); // �������ͨ����־�����跢��ͨ����ͨ��4��

    // ʹ�ܽ���ͨ���Ĵ�������жϣ���ѡ��
    DMA_ITConfig(USART_RX_DMA_CHANNEL, DMA_IT_TE, ENABLE);

    // ʹ�ܽ���DMA
    DMA_Cmd(USART_RX_DMA_CHANNEL, ENABLE);
    // ����DMA��ʱ��ʹ�ܣ��ڷ���ʱ��ʹ��
	
	
	
}




//DMA��������
void USARTx_DMA_Rx_Config(void)
{
	
		// �ر�DMA ����ֹ����
	DMA_Cmd(USART_RX_DMA_CHANNEL, DISABLE); 
  //����
	USART_RX_DMA_CHANNEL->CMAR=(uint32_t)ucRTUBuf;              // �ڴ��ַ�����ջ�������
	   // ���ô��䳤��
    USART_RX_DMA_CHANNEL->CNDTR = USART_RBUFF_SIZE; 
	// ��DMA��־λ
	DMA_ClearFlag( DMA1_FLAG_TC5|DMA1_FLAG_HT5|DMA1_FLAG_GL5 );          
	//  ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
	USART_RX_DMA_CHANNEL->CNDTR = USART_RBUFF_SIZE; 
  	USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, ENABLE); 
	DMA_Cmd(USART_RX_DMA_CHANNEL, ENABLE);

}
//DMA��������
void USARTx_DMA_Tx_Config(uint8_t *pData, uint16_t len)
{
	
	// �ȹر�DMA
    DMA_Cmd(USART_TX_DMA_CHANNEL, DISABLE);
    // �����ڴ��ַ
     USART_TX_DMA_CHANNEL->CMAR= (uint32_t)pData;
    // ���ô��䳤��
    USART_TX_DMA_CHANNEL->CNDTR = len; 
   
    // �����־λ�����ݷ���ͨ����ʵ�ʱ�־λ�����
  	DMA_ClearFlag( DMA1_FLAG_TC5|DMA1_FLAG_HT5|DMA1_FLAG_GL5 );  // ���跢��ͨ����ͨ��4
	
	// ʹ�ܽ���ͨ���Ĵ�������жϣ���ѡ��
    DMA_ITConfig(USART_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
    // ʹ��DMA
    DMA_Cmd(USART_TX_DMA_CHANNEL, ENABLE);
	
	
}



extern SemaphoreHandle_t BinarySem_Handle;

//void Uart_DMA_Rx_Data(void)
//{
//	BaseType_t pxHigherPriorityTaskWoken;
//	// �ر�DMA ����ֹ����
//	DMA_Cmd(USART_RX_DMA_CHANNEL, DISABLE);      
//	// ��DMA��־λ
//	DMA_ClearFlag( DMA1_FLAG_TC5 );          
//	//  ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
//	USART_RX_DMA_CHANNEL->CNDTR = USART_RBUFF_SIZE;    
////	DMA_Cmd(USART_RX_DMA_CHANNEL, ENABLE); //��postserial.c��ʹ�ܴ򿪣�����modbusrtu
//	
//	/* 
//	xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore,
//												BaseType_t *pxHigherPriorityTaskWoken);
//	*/
//	
//	//������ֵ�ź��� �����ͽ��յ������ݱ�־����ǰ̨�����ѯ
//	xSemaphoreGiveFromISR(BinarySem_Handle,&pxHigherPriorityTaskWoken);	//�ͷŶ�ֵ�ź���
//  //�����Ҫ�Ļ�����һ�������л���ϵͳ���ж��Ƿ���Ҫ�����л�
//	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

//}

/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
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

