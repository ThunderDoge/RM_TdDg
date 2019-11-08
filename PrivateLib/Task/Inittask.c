/** 
* @brief    ��ʼ������
* @details  �������񣬳�ʼ���弶Ӳ��
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/
#include "Inittask.h"

EventGroupHandle_t Inittask_Event_Handle = NULL; /*��ʼ���¼����*/

TaskHandle_t Init_Task_Handle = NULL;	/*��ʼ��������*/
TaskHandle_t AppTaskCreat_Handle = NULL; /*���񴴽����*/

/**
* @brief  ���񴴽�����
* @details  �����ٽ������������������˳�����ʼ�������
* @param  NULL
* @retval  NULL
*/
void Task_CreateTasks(void)
{
	taskENTER_CRITICAL();	/*�����ٽ���*/
	
	//������д�ģ�����Ϊ�������ź�������ע�͵���
//	Inittask_Event_Handle = xEventGroupCreate(); /*������ʼ������ж��¼�*/
//	Dbus_Update_Handle = xSemaphoreCreateBinary(); /*����Dbus���ݸ��¶�ֵ������*/
//	CAN_Update_Handle = xSemaphoreCreateBinary(); /*����CAN���ݸ��¶�ֵ������*/
//	Dbus_Check_Handle = xSemaphoreCreateBinary(); /*����Dbus���߼���ֵ�ź���*/
//	CAN_Check_Handle = xSemaphoreCreateBinary(); /*����CAN���߼���ֵ�ź���*/
	
		bsp_CAN_Init(); /*��ʼ��CAN����*/
		bsp_Dbus_Init(); /*��ʼ��Dbus����*/

		Music_Play(INTEL); /*���ſ������֣��Լ����֮��ʼ����������*/
		//Music_Play(jile);
		vTaskDelay(1000); /*��ʱһ�룬�ȴ�ϵͳ�ȶ�*/


//	xTaskCreate((TaskFunction_t )Log_Task,
//							(const char*    )"Log_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )1,
//							(TaskHandle_t*  )&Log_Task_Handle); /*����ϵͳ״̬��������*/
							
//	xTaskCreate((TaskFunction_t )Control_Task,
//							(const char*    )"Control_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )2,
//							(TaskHandle_t*  )&Control_Task_Handle);	/*������������*/
	xTaskCreate((TaskFunction_t )task_test_control,
							(const char*    )"Control_Task",
							(uint16_t       )512,
							(void*          )NULL,
							(UBaseType_t    )2,
							(TaskHandle_t*  )&Control_Task_Handle);	/*������������*/
							
//	xTaskCreate((TaskFunction_t )IMU_Task,
//							(const char*    )"IMU_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )3,
//							(TaskHandle_t*  )&IMU_Task_Handle); /*������̬��������*/
							
//	xTaskCreate((TaskFunction_t )BspInit_Task,
//							(const char*    )"BspInit_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )4,
//							(TaskHandle_t*  )&Init_Task_Handle); /*���������ʼ������*/

	vTaskDelete(NULL); /*ɾ�����񴴽�����*/
	
	taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}

/**
* @brief  �弶Ӳ����ʼ������
* @details  ��ʼ�����Ӳ������ɳ�ʼ��֮������¼�����������
* @param  ��ʼ��������
* @retval  NULL
*/
void BspInit_Task(void* pvParameters)
{
//		bsp_MPU9250_Init();
//		IMU_Init();
//		bsp_led_Init(); /*��ʼ��LED��������*/
		bsp_CAN_Init(); /*��ʼ��CAN����*/
		bsp_Dbus_Init(); /*��ʼ��Dbus����*/

		Music_Play(INTEL); /*���ſ������֣��Լ����֮��ʼ����������*/
		//Music_Play(jile);
		vTaskDelay(1000); /*��ʱһ�룬�ȴ�ϵͳ�ȶ�*/
//		xEventGroupSetBits(Inittask_Event_Handle,Init_Event); /*�����¼���ϵͳ��ʼ���Ѿ����*/
		
		vTaskDelete(Init_Task_Handle); /*ɾ����ʼ������*/
}