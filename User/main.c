/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   FreeRTOS V9.0.0  + STM32 �жϹ���
  *********************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/ 
/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ������Ӳ��bspͷ�ļ� */
#include "bsp_led.h"
#include "bsp_usart.h"

/* ��׼��ͷ�ļ� */
#include <string.h>
//#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
//	#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
/* ----------------------- Modbus ���� ----------------------------------*/ 
#include "mb.h"
#include "user_mb_app.h"
/**************************** ������ ********************************/
/* 
 * ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 * �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 * ����������ΪNULL��
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* ���������� */
static TaskHandle_t Modbus_Task_Handle = NULL;/* KEY������ */

/********************************** �ں˶����� *********************************/
/*
 * �ź�������Ϣ���У��¼���־�飬�����ʱ����Щ�������ں˵Ķ���Ҫ��ʹ����Щ�ں�
 * ���󣬱����ȴ����������ɹ�֮��᷵��һ����Ӧ�ľ����ʵ���Ͼ���һ��ָ�룬������
 * �ǾͿ���ͨ��������������Щ�ں˶���
 *
 * �ں˶���˵���˾���һ��ȫ�ֵ����ݽṹ��ͨ����Щ���ݽṹ���ǿ���ʵ��������ͨ�ţ�
 * �������¼�ͬ���ȸ��ֹ��ܡ�������Щ���ܵ�ʵ��������ͨ��������Щ�ں˶���ĺ���
 * ����ɵ�
 * 
 */
SemaphoreHandle_t BinarySem_Handle =NULL;

/******************************* ȫ�ֱ������� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */
 
 
 
/******************************* �궨�� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩ�궨�塣
 */

/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */

//static void Modbus_Task(void* pvParameters);/* KEY_Task����ʵ�� */

static void Modbus_Task(void);/* KEY_Task����ʵ�� */
static void BSP_Init(void);/* ���ڳ�ʼ�����������Դ */

/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   ��һ����������Ӳ����ʼ�� 
            �ڶ���������APPӦ������
            ������������FreeRTOS����ʼ���������
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
  /* ������Ӳ����ʼ�� */
  BSP_Init();
  
	printf("����һ��freemodbus��dma�汾��\n");
   Modbus_Task();
 
//   /* ����AppTaskCreate���� */
//  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
//                        (const char*    )"AppTaskCreate",/* �������� */
//                        (uint16_t       )512,  /* ����ջ��С */
//                        (void*          )NULL,/* ������ں������� */
//                        (UBaseType_t    )1, /* ��������ȼ� */
//                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 
//  /* ����������� */           
//  if(pdPASS == xReturn)
//    vTaskStartScheduler();   /* �������񣬿������� */
//  else
//    return -1;  

  while(1);/* ��������ִ�е����� */    
}


/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
  taskENTER_CRITICAL();           //�����ٽ���
	
  /* ���� BinarySem */
  BinarySem_Handle = xSemaphoreCreateBinary();	 
  
	if(NULL != BinarySem_Handle)
    printf("BinarySem_Handle��ֵ�ź��������ɹ�!\n");
	
  /* Modbus_Task */
  xReturn = xTaskCreate((TaskFunction_t )Modbus_Task,  /* ������ں��� */
                        (const char*    )"Modbus_Task",/* �������� */
                        (uint16_t       )1024,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&Modbus_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("����Modbus_Task����ɹ�!\n");
  
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}



/**********************************************************************
  * @ ������  �� LED_Task
  * @ ����˵���� LED_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
//static void Modbus_Task(void* parameter)
	static void Modbus_Task(void)
{	
	int i;

 /* ѡ�� ASCII �� RTU ģʽ�� 
   
*/
	eMBInit (MB_RTU , 0x0A,  0 , 115200,  MB_PAR_EVEN );
		//������  * Modbus ��ջ  ֮ǰ��ʼ�����ּĴ���ֵ 
 
    for ( i = 0; i < S_REG_HOLDING_NREGS; i++ )
    {
        usSRegHoldBuf[i] = (  unsigned  short )i;
    }
//     
///* ������  Modbus ��ջ ֮ǰ��ʼ������Ĵ���ֵ 
// 
//		*/ 
    for (i = 0; i < S_REG_INPUT_NREGS; i++ )
    {
        usSRegInBuf[i] = ( unsigned  short)i;
    }

 /* ���� Modbus Э��ջ�� 
  
		*/ 
		eMBEnable();
     for ( ; ; )
    {
         /* ���� Modbus Э��ջ������ѯѭ����*/ 
         eMBPoll();
    }
}

/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� �弶�����ʼ�������а����ϵĳ�ʼ�����ɷ��������������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
	 * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
	 * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
	 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* LED ��ʼ�� */
	LED_GPIO_Config();


	/* DMA��ʼ��	*/
	USARTx_DMA_Config();

	
	/* ���ڳ�ʼ��	*/
	USART_Config();
	
	
	
}

/********************************END OF FILE****************************/
