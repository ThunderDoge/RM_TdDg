/**
 * @file task_SentiCloud.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Tasks of SentryCloud
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __TASK_SENTI_CLOUD_H_
#define __TASK_SENTI_CLOUD_H_

#include "sentry_cloud_config.h"

#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "app_imu.h"
#include "app_check.h"
#include "Sentry.hpp"

extern TaskHandle_t task_Main_Handle,task_CommuRoutine_Handle,task_Check_Handle;
extern uint32_t mark1, mark2, mark3;


void Cloud_Init(void);	/// 硬件初始化
void TaskStarter(void); /// 全任务启动器


#endif // __TASK_SENTI_CLOUD_H_
