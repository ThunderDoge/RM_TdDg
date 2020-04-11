/**
 * @file task_SentryDownCloud.hpp
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

#ifndef __PROJECT_SENTRY_DOWN_CLOUD_     //定义工程标识符__PROJECT_SENTRY_DOWN_CLOUD_
#define __PROJECT_SENTRY_DOWN_CLOUD_
#endif // __PROJECT_SENTRY_DOWN_CLOUD_

// 依赖的文件

//系统驱动
#include "cmsis_os.h"

//硬件驱动
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "bsp_spi.h"
#include "app_imu.h"

//机器人逻辑
#include "Sentry.hpp"

void DownCloud_Init(void);  /// 硬件初始化
void TaskStarter(void); /// 全任务启动器

#endif // __TASK_SENTI_CLOUD_H_
