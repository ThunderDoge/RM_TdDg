#ifndef __INIT_TASK_H_
#define __INIT_TASK_H_

#include "task_config.h"
#include "bsp_config.h"
#include "app_config.h"

extern EventGroupHandle_t Inittask_Event_Handle; /*��ʼ���¼����*/

extern TaskHandle_t Init_Task_Handle;	/*��ʼ��������*/
extern TaskHandle_t AppTaskCreat_Handle; /*���񴴽����*/


void BspInit_Task(void* pvParameters);
void Task_CreateTasks(void);	/*ϵͳ��ʼ����ǰ�������񴴽�*/

#endif
