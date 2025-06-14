/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* ������Ӳ��bspͷ�ļ� */
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h"



/* USER CODE BEGIN 0 */
#include "mb.h"
#include "mbport.h"

#include "stdlib.h"

extern uint16_t mbrec_timeout;
extern volatile UCHAR  ucRTUBuf[256];
extern volatile USHORT usRcvBufferlen;//֡�����ֽڳ���
extern EventGroupHandle_t xSlaveOsEvent;

extern unsigned char xSlaveState;//״̬

uint16_t BASIC_TIM_num=0;
uint16_t	last_frame_time=0;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void xPortSysTickHandler(void);
//systick�жϷ�����
void SysTick_Handler(void)
{	
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif  /* INCLUDE_xTaskGetSchedulerState */  
    
    xPortSysTickHandler();
    
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
  }
#endif  /* INCLUDE_xTaskGetSchedulerState */
}



/* ���������ⲿ���� & ��ֵ�ź��� */
extern SemaphoreHandle_t BinarySem_Handle;

/*********************************************************************************
  * @ ������  �� DEBUG_USART_IRQHandler
  * @ ����˵���� �����жϷ�����
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  ********************************************************************************/
uint32_t  tttttt=0;
void DEBUG_USART_IRQHandler(void)
{
  uint32_t ulReturn;
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken=pdFALSE;

  /* �����ٽ�Σ��ٽ�ο���Ƕ�� */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();

	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_IDLE)!=RESET)
	{		
		 if((BASIC_TIM_num-last_frame_time >mbrec_timeout))
		 {
			 
//					xResult=xEventGroupSetBitsFromISR(xSlaveOsEvent, EV_FRAME_RECEIVED,&xHigherPriorityTaskWoken);//���ͽ�������¼�
//					if ( xResult != pdFAIL )
//					{
//						 /* ��� xHigherPriorityTaskWoken ��ֵΪ pdTRUE
//						 �����һ���������л�*/
//						 portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
//					}
					
					xSlaveState=EV_FRAME_RECEIVED;
//			 DMA_Cmd(USART_RX_DMA_CHANNEL, DISABLE);
					usRcvBufferlen=256-DMA_GetCurrDataCounter(USART_RX_DMA_CHANNEL);
					//�رս���dma
		 }
		 else
			{
							memset(ucRTUBuf,0,256);/* �������쳣ʱ�����һ�㲻�ᵽ�� */
			}
//		Uart_DMA_Rx_Data();       /* �ͷ�һ���ź�������ʾ�����ѽ��� */
			//�رմ��ڽ���
				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, DISABLE); 
			USART_ReceiveData(DEBUG_USARTx); /* �����־λ */
			last_frame_time = BASIC_TIM_num;

	}	 
  
  /* �˳��ٽ�� */
//  taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}
/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void  BASIC_TIM_IRQHandler (void)
{
		
	 uint32_t ulReturn;
	  /* �����ٽ�Σ��ٽ�ο���Ƕ�� */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
	{	
		
			TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  
	  	BASIC_TIM_num++;
		 if(BASIC_TIM_num>=65534)
		 {
				BASIC_TIM_num=BASIC_TIM_num-last_frame_time;//������Ĳ�д�룬��BASIC_TIM_numһֱ����last_frame_time
			 last_frame_time=0;//��last_frame_time�������ͷ�ж�
			 
		 }
			
	}
			 /* �˳��ٽ�� */
//  taskEXIT_CRITICAL_FROM_ISR( ulReturn );	
}
unsigned char ttt=0;
void DMA1_Channel4_IRQHandler()
{
	
	 uint32_t ulReturn;
		BaseType_t xHigherPriorityTaskWoken, xResult;
	  /* �����ٽ�Σ��ٽ�ο���Ƕ�� */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();
		if (DMA_GetITStatus(DMA1_IT_TC4))
			{
					// �ر�DMA ����ֹ����
					DMA_Cmd(USART_TX_DMA_CHANNEL, DISABLE); 
//					xResult=xEventGroupSetBitsFromISR(xSlaveOsEvent, EV_FRAME_SENT,&xHigherPriorityTaskWoken);//���ͽ�������¼�
//					if ( xResult != pdFAIL )
//					{
//						 /* ��� xHigherPriorityTaskWoken ��ֵΪ pdTRUE
//						 �����һ���������л�*/
//						 portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
//					}
				
					xSlaveState=EV_FRAME_SENT;
						DMA_ClearFlag( DMA1_FLAG_TC4|DMA1_FLAG_HT4|DMA1_FLAG_GL4 );
	
			}
			 /* �˳��ٽ�� */
//  taskEXIT_CRITICAL_FROM_ISR( ulReturn );
	
}
