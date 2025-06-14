/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* 开发板硬件bsp头文件 */
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h"



/* USER CODE BEGIN 0 */
#include "mb.h"
#include "mbport.h"

#include "stdlib.h"

extern uint16_t mbrec_timeout;
extern volatile UCHAR  ucRTUBuf[256];
extern volatile USHORT usRcvBufferlen;//帧接收字节长度
extern EventGroupHandle_t xSlaveOsEvent;

extern unsigned char xSlaveState;//状态

uint16_t BASIC_TIM_num=0;
uint16_t	last_frame_time=0;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void xPortSysTickHandler(void);
//systick中断服务函数
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



/* 声明引用外部队列 & 二值信号量 */
extern SemaphoreHandle_t BinarySem_Handle;

/*********************************************************************************
  * @ 函数名  ： DEBUG_USART_IRQHandler
  * @ 功能说明： 串口中断服务函数
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  ********************************************************************************/
uint32_t  tttttt=0;
void DEBUG_USART_IRQHandler(void)
{
  uint32_t ulReturn;
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken=pdFALSE;

  /* 进入临界段，临界段可以嵌套 */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();

	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_IDLE)!=RESET)
	{		
		 if((BASIC_TIM_num-last_frame_time >mbrec_timeout))
		 {
			 
//					xResult=xEventGroupSetBitsFromISR(xSlaveOsEvent, EV_FRAME_RECEIVED,&xHigherPriorityTaskWoken);//发送接收完成事件
//					if ( xResult != pdFAIL )
//					{
//						 /* 如果 xHigherPriorityTaskWoken 的值为 pdTRUE
//						 则进行一次上下文切换*/
//						 portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
//					}
					
					xSlaveState=EV_FRAME_RECEIVED;
//			 DMA_Cmd(USART_RX_DMA_CHANNEL, DISABLE);
					usRcvBufferlen=256-DMA_GetCurrDataCounter(USART_RX_DMA_CHANNEL);
					//关闭接收dma
		 }
		 else
			{
							memset(ucRTUBuf,0,256);/* 当数据异常时清除，一般不会到这 */
			}
//		Uart_DMA_Rx_Data();       /* 释放一个信号量，表示数据已接收 */
			//关闭串口接收
				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, DISABLE); 
			USART_ReceiveData(DEBUG_USARTx); /* 清除标志位 */
			last_frame_time = BASIC_TIM_num;

	}	 
  
  /* 退出临界段 */
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
	  /* 进入临界段，临界段可以嵌套 */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
	{	
		
			TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  
	  	BASIC_TIM_num++;
		 if(BASIC_TIM_num>=65534)
		 {
				BASIC_TIM_num=BASIC_TIM_num-last_frame_time;//将溢出的差写入，让BASIC_TIM_num一直大于last_frame_time
			 last_frame_time=0;//将last_frame_time清除，重头判断
			 
		 }
			
	}
			 /* 退出临界段 */
//  taskEXIT_CRITICAL_FROM_ISR( ulReturn );	
}
unsigned char ttt=0;
void DMA1_Channel4_IRQHandler()
{
	
	 uint32_t ulReturn;
		BaseType_t xHigherPriorityTaskWoken, xResult;
	  /* 进入临界段，临界段可以嵌套 */
//  ulReturn = taskENTER_CRITICAL_FROM_ISR();
		if (DMA_GetITStatus(DMA1_IT_TC4))
			{
					// 关闭DMA ，防止干扰
					DMA_Cmd(USART_TX_DMA_CHANNEL, DISABLE); 
//					xResult=xEventGroupSetBitsFromISR(xSlaveOsEvent, EV_FRAME_SENT,&xHigherPriorityTaskWoken);//发送接收完成事件
//					if ( xResult != pdFAIL )
//					{
//						 /* 如果 xHigherPriorityTaskWoken 的值为 pdTRUE
//						 则进行一次上下文切换*/
//						 portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
//					}
				
					xSlaveState=EV_FRAME_SENT;
						DMA_ClearFlag( DMA1_FLAG_TC4|DMA1_FLAG_HT4|DMA1_FLAG_GL4 );
	
			}
			 /* 退出临界段 */
//  taskEXIT_CRITICAL_FROM_ISR( ulReturn );
	
}
