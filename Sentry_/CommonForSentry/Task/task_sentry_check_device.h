/**
  * @file      task_sentry_check_device.h
  * @brief     公用设备离线检测任务
  * @details   
  * @author   ThunderDoge
  * @date      2020-3-18
  * @version   1.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */

#ifndef __TASK_SENTRY_CHECK_DEVICE_H
#define __TASK_SENTRY_CHECK_DEVICE_H

//依赖的文件

#include "cmsis_os.h"
#include "app_sentry_check_device.hpp"

extern TaskHandle_t task_CheckDevice_Handle;

void task_CheckDevice(void* param);

#endif // __TASK_SENTRY_CHECK_DEVICE_H



 