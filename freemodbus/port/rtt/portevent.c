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
 * File: $Id: portevent.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "event_groups.h"
/* ----------------------- Variables ----------------------------------------*/
EventGroupHandle_t xSlaveOsEvent =NULL;

unsigned char xSlaveState=0;//状态
//static struct rt_event     xSlaveOsEvent;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{

//		xSlaveOsEvent = xEventGroupCreate();	
			xSlaveState=0;//状态
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
//    rt_event_send(&xSlaveOsEvent, eEvent);
	       /* start serial transmit */
//        xEventGroupSetBits(xSlaveOsEvent, eEvent);
			xSlaveState=eEvent;//状态
    return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    uint32_t recvedEvent;
    /* waiting forever OS event */
	
//		 recvedEvent = xEventGroupWaitBits(xSlaveOsEvent,  /* 事件对象句柄 */
//                                  EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT,/* 接收线程感兴趣的事件 */
//                                  pdTRUE,   /* 退出时清除事件位 */
//                                  pdFALSE,   /* 满足感兴趣的一个事件 */
//                                  100 );/* 指定超时事件,一直等 */

	
	recvedEvent=xSlaveState;
	xSlaveState=0;
    switch (recvedEvent)
    {
    case EV_READY:
        *eEvent = EV_READY;
        break;
    case EV_FRAME_RECEIVED:
        *eEvent = EV_FRAME_RECEIVED;
        break;
    case EV_EXECUTE:
        *eEvent = EV_EXECUTE;
        break;
    case EV_FRAME_SENT:
        *eEvent = EV_FRAME_SENT;
        break;
    }
    return TRUE;
}
