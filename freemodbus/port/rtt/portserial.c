/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"
#include "FreeRTOS.h"
#include "event_groups.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "bsp_usart.h"

/*----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

static EventGroupHandle_t event_serial =NULL;
extern volatile UCHAR  ucRTUBuf[256];
/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
static void serial_soft_trans_irq(void* parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
    	/* DMA��ʼ��	*/
	USARTx_DMA_Rx_Config();
	
	/* ���ڳ�ʼ��	*/
	USART_Config();
	
	  /* �¼�����  */
//		event_serial = xEventGroupCreate();	
    return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable,USHORT TX_n)
{
    uint32_t recved_event;
    if (xRxEnable)
    {
					
//			// ���DMA���б�־
//			DMA_ClearFlag(DMA1_FLAG_GL5);
				/* ���ڳ�ʼ��	*/

			ENTER_CRITICAL_SECTION();
				// �ر�DMA ����ֹ����
				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, ENABLE); 
					 USARTx_DMA_Rx_Config();
			EXIT_CRITICAL_SECTION() ;			
//				// ��DMA��־λ
//				DMA_ClearFlag( DMA1_FLAG_TC5 );
			
			
				// ���� ���ڿ���IDEL �ж�
//				USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, ENABLE);  
//				// ��������DMA����
//				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, ENABLE); 
//				// ʹ��DMA
//				DMA_Cmd (USART_RX_DMA_CHANNEL,ENABLE);
      
    }
    else
    {
				// �رմ��ڿ���IDEL �ж�
//				USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, DISABLE);  
//				// �رմ���DMA����
//				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, DISABLE); 
//				// ��ʹ��DMA
//				DMA_Cmd (USART_RX_DMA_CHANNEL,DISABLE);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
//			USART_Config();
				// �رմ���DMA����	
			// 1. ���� DMA ͨ�����ؼ����裡��
			ENTER_CRITICAL_SECTION();
			
//			USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Rx, DISABLE); 
	    USARTx_DMA_Tx_Config(ucRTUBuf,TX_n);//TX_n
		
			EXIT_CRITICAL_SECTION() ;
   
			
//			DMA_ClearFlag( DMA1_FLAG_TC5 );
			
			//����dma����
			  // ��������DMA����
//				USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Tx, ENABLE);
//			// ʹ��DMA
//			DMA_Cmd (USART_TX_DMA_CHANNEL,ENABLE);
			 
    }
    else
    {
//			 recved_event = xEventGroupWaitBits(event_serial,  /* �¼������� */
//                                  EVENT_SERIAL_TRANS_START,/* �����̸߳���Ȥ���¼� */
//                                  pdTRUE,   /* �˳�ʱ����¼�λ */
//                                  pdFALSE,   /* �������Ȥ��һ���¼� */
//                                  0);/* ָ����ʱ�¼�,һֱ�� */
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
//                &recved_event);
    }
}

void vMBPortClose(void)
{
//    serial->parent.close(&(serial->parent));
		// �رմ���
	USART_Cmd(DEBUG_USARTx, DISABLE);
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
//    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
	Usart_SendByte( USART1, ucByte);
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
//    serial->parent.read(&(serial->parent), 0, pucByte, 1);
    return TRUE;
}

/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
//    pxMBFrameCBTransmitterEmpty();
}

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
//    pxMBFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void* parameter) {
//    rt_uint32_t recved_event;
//    while (1)
//    {
//        /* waiting for serial transmit start */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
//                RT_WAITING_FOREVER, &recved_event);
//        /* execute modbus callback */
//        prvvUARTTxReadyISR();
//    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size) {
////    prvvUARTRxISR();
////    return RT_EOK;
//}
