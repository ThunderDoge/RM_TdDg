#ifndef __APP_LOG_H_
#define __APP_LOG_H_

#include "bsp_config.h"
#include "app_config.h"
#include "task_config.h"

extern TaskHandle_t Log_Task_Handle;	/*系统状态任务句柄*/

void Log_Task(void* pvParameters);

#endif
