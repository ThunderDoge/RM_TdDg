#ifndef __TASK_IMU_H
#define __TASK_IMU_H

#include "bsp_mpu9250.h"
#include "app_config.h"
#include "task_config.h"

extern TaskHandle_t IMU_Task_Handle;

void IMU_Task(void* pvParameters);

#endif
