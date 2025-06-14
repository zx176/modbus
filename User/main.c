/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   FreeRTOS V9.0.0  + STM32 中断管理
  *********************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 开发板硬件bsp头文件 */
#include "bsp_led.h"
#include "bsp_usart.h"

/* 标准库头文件 */
#include <string.h>
//#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
//	#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
/* ----------------------- Modbus 包括 ----------------------------------*/ 
#include "mb.h"
#include "user_mb_app.h"
/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t Modbus_Task_Handle = NULL;/* KEY任务句柄 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */
SemaphoreHandle_t BinarySem_Handle =NULL;

/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
 
 
 
/******************************* 宏定义 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些宏定义。
 */

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

//static void Modbus_Task(void* pvParameters);/* KEY_Task任务实现 */

static void Modbus_Task(void);/* KEY_Task任务实现 */
static void BSP_Init(void);/* 用于初始化板载相关资源 */

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  /* 开发板硬件初始化 */
  BSP_Init();
  
	printf("这是一个freemodbus的dma版本！\n");
   Modbus_Task();
 
//   /* 创建AppTaskCreate任务 */
//  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
//                        (const char*    )"AppTaskCreate",/* 任务名字 */
//                        (uint16_t       )512,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )1, /* 任务的优先级 */
//                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
//  /* 启动任务调度 */           
//  if(pdPASS == xReturn)
//    vTaskStartScheduler();   /* 启动任务，开启调度 */
//  else
//    return -1;  

  while(1);/* 正常不会执行到这里 */    
}


/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  taskENTER_CRITICAL();           //进入临界区
	
  /* 创建 BinarySem */
  BinarySem_Handle = xSemaphoreCreateBinary();	 
  
	if(NULL != BinarySem_Handle)
    printf("BinarySem_Handle二值信号量创建成功!\n");
	
  /* Modbus_Task */
  xReturn = xTaskCreate((TaskFunction_t )Modbus_Task,  /* 任务入口函数 */
                        (const char*    )"Modbus_Task",/* 任务名字 */
                        (uint16_t       )1024,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&Modbus_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建Modbus_Task任务成功!\n");
  
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}



/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
//static void Modbus_Task(void* parameter)
	static void Modbus_Task(void)
{	
	int i;

 /* 选择 ASCII 或 RTU 模式。 
   
*/
	eMBInit (MB_RTU , 0x0A,  0 , 115200,  MB_PAR_EVEN );
		//在启动  * Modbus 堆栈  之前初始化保持寄存器值 
 
    for ( i = 0; i < S_REG_HOLDING_NREGS; i++ )
    {
        usSRegHoldBuf[i] = (  unsigned  short )i;
    }
//     
///* 在启动  Modbus 堆栈 之前初始化输入寄存器值 
// 
//		*/ 
    for (i = 0; i < S_REG_INPUT_NREGS; i++ )
    {
        usSRegInBuf[i] = ( unsigned  short)i;
    }

 /* 启用 Modbus 协议栈。 
  
		*/ 
		eMBEnable();
     for ( ; ; )
    {
         /* 调用 Modbus 协议栈的主轮询循环。*/ 
         eMBPoll();
    }
}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* LED 初始化 */
	LED_GPIO_Config();


	/* DMA初始化	*/
	USARTx_DMA_Config();

	
	/* 串口初始化	*/
	USART_Config();
	
	
	
}

/********************************END OF FILE****************************/
