/**
  * @file      task_sentry_check_device.c
  * @brief     �����豸���߼������
  * @details   
  * @author   ThunderDoge
  * @date      2020-3-18
  * @version   1.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */

#include "task_sentry_check_device.h"

#define TASK_CHECKDEVICE_DELAY_TIME_MS 10

TaskHandle_t task_CheckDevice_Handle;
uint32_t mark_check_device;

/**
 * @brief �����豸���߼������
 * 
 * @param     param = NULL ����
 */
void task_CheckDevice(void* param)
{
	TickType_t LastTick = xTaskGetTickCount();
	while(1)
	{
		app_sentry_CheckDevice_TaskHandler();
		vTaskDelayUntil( &LastTick , TASK_CHECKDEVICE_DELAY_TIME_MS / portTICK_PERIOD_MS );
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark_check_device = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}


