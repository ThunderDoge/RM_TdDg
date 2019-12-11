#ifndef __TASK_SENTI_CLOUD_H_
#define __TASK_SENTI_CLOUD_H_

#include "can.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "bsp_spi.h"
#include "bsp_vision.hpp"
#include "app_imu.h"
#include "app_AmmoFeed.hpp"
#include "SentryCloud.hpp"
#include "Sentry.hpp"

void TaskStarter(void);

// void task_CloudCtrl(void* param);
// void task_CloudMotorManage(void* param);
// void task_VisionUart(void* param);
// void task_Commander(void* param);
// void task_ImuUpdate(void* param);
// void RecvFromCan(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef* RxHead,uint8_t* Data);
// void task_can_test(void*param);
#endif // __TASK_SENTI_CLOUD_H_
