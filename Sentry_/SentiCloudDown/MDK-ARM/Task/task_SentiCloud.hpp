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

#ifndef __PROJECT_SENTRY_CLOUD_     //定义工程标识符__PROJECT_SENTRY_CLOUD_
#define __PROJECT_SENTRY_CLOUD_
#endif // __PROJECT_SENTRY_CLOUD_


//#include "can.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "bsp_spi.h"
// #include "app_vision.hpp"
// #include "app_AmmoFeed.hpp"
// #include "SentryCloud.hpp"
#include "cmsis_os.h"
#include "app_imu.h"
#include "Sentry.hpp"
#include "task_sentry_check_device.h"

void TaskStarter(void); ///全任务启动器

// void task_CloudCtrl(void* param);
// void task_CloudMotorManage(void* param);
// void task_VisionUart(void* param);
// void task_Commander(void* param);
// void task_ImuUpdate(void* param);
// void RecvFromCan(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef* RxHead,uint8_t* Data);
// void task_can_test(void*param);
#endif // __TASK_SENTI_CLOUD_H_
