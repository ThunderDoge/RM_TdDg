#ifndef __INIT_TASK_H_
#define __INIT_TASK_H_

#include "task_config.h"
#include "bsp_config.h"
#include "app_config.h"

extern EventGroupHandle_t Inittask_Event_Handle; /*初始化事件句柄*/

extern TaskHandle_t Init_Task_Handle;	/*初始化任务句柄*/
extern TaskHandle_t AppTaskCreat_Handle; /*任务创建句柄*/


void BspInit_Task(void* pvParameters);
void Task_CreateTasks(void);	/*系统开始调度前负责任务创建*/

#endif
