#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "app_pid.h"
#include "app_quad_math.h"
#include "app_chassis.h"
#include "task_config.h"

extern TaskHandle_t Control_Task_Handle;
extern SemaphoreHandle_t Dbus_Update_Handle; /*Dbus二值化更新变量*/
extern SemaphoreHandle_t CAN_Update_Handle; /*CAN二值化更新变量*/

void Control_Task(void* pvParameters);
void task_test_control(void* param);

#endif
