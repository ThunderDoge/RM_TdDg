/** 
* @brief    初始化任务
* @details  创建任务，初始化板级硬件
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/
#include "Inittask.h"

EventGroupHandle_t Inittask_Event_Handle = NULL; /*初始化事件句柄*/

TaskHandle_t Init_Task_Handle = NULL;	/*初始化任务句柄*/
TaskHandle_t AppTaskCreat_Handle = NULL; /*任务创建句柄*/

/**
* @brief  任务创建任务
* @details  进入临界区创建相关任务，随后退出，开始任务调度
* @param  NULL
* @retval  NULL
*/
void Task_CreateTasks(void)
{
	taskENTER_CRITICAL();	/*进入临界区*/
	
	//郭俊辉写的，玉因为不会用信号量所以注释掉了
//	Inittask_Event_Handle = xEventGroupCreate(); /*创建初始化完成判断事件*/
//	Dbus_Update_Handle = xSemaphoreCreateBinary(); /*创建Dbus数据更新二值化变量*/
//	CAN_Update_Handle = xSemaphoreCreateBinary(); /*创建CAN数据更新二值化变量*/
//	Dbus_Check_Handle = xSemaphoreCreateBinary(); /*创建Dbus离线检测二值信号量*/
//	CAN_Check_Handle = xSemaphoreCreateBinary(); /*创建CAN离线检测二值信号量*/
	
		bsp_CAN_Init(); /*初始化CAN总线*/
		bsp_Dbus_Init(); /*初始化Dbus串口*/

		Music_Play(INTEL); /*播放开机音乐，自检完成之后开始更新任务句柄*/
		//Music_Play(jile);
		vTaskDelay(1000); /*延时一秒，等待系统稳定*/


//	xTaskCreate((TaskFunction_t )Log_Task,
//							(const char*    )"Log_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )1,
//							(TaskHandle_t*  )&Log_Task_Handle); /*创建系统状态报告任务*/
							
//	xTaskCreate((TaskFunction_t )Control_Task,
//							(const char*    )"Control_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )2,
//							(TaskHandle_t*  )&Control_Task_Handle);	/*创建控制任务*/
	xTaskCreate((TaskFunction_t )task_test_control,
							(const char*    )"Control_Task",
							(uint16_t       )512,
							(void*          )NULL,
							(UBaseType_t    )2,
							(TaskHandle_t*  )&Control_Task_Handle);	/*创建控制任务*/
							
//	xTaskCreate((TaskFunction_t )IMU_Task,
//							(const char*    )"IMU_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )3,
//							(TaskHandle_t*  )&IMU_Task_Handle); /*创建姿态解算任务*/
							
//	xTaskCreate((TaskFunction_t )BspInit_Task,
//							(const char*    )"BspInit_Task",
//							(uint16_t       )512,
//							(void*          )NULL,
//							(UBaseType_t    )4,
//							(TaskHandle_t*  )&Init_Task_Handle); /*创建外设初始化任务*/

	vTaskDelete(NULL); /*删除任务创建任务*/
	
	taskEXIT_CRITICAL();	/*退出临界区*/
}

/**
* @brief  板级硬件初始化任务
* @details  初始化相关硬件，完成初始化之后更新事件，销毁任务
* @param  初始化任务句柄
* @retval  NULL
*/
void BspInit_Task(void* pvParameters)
{
//		bsp_MPU9250_Init();
//		IMU_Init();
//		bsp_led_Init(); /*初始化LED引脚配置*/
		bsp_CAN_Init(); /*初始化CAN总线*/
		bsp_Dbus_Init(); /*初始化Dbus串口*/

		Music_Play(INTEL); /*播放开机音乐，自检完成之后开始更新任务句柄*/
		//Music_Play(jile);
		vTaskDelay(1000); /*延时一秒，等待系统稳定*/
//		xEventGroupSetBits(Inittask_Event_Handle,Init_Event); /*更新事件，系统初始化已经完成*/
		
		vTaskDelete(Init_Task_Handle); /*删除初始化任务*/
}