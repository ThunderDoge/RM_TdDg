#ifndef __SENTRY_CLOUD_CAN_COMMU_HPP_
#define __SENTRY_CLOUD_CAN_COMMU_HPP_


/** 
 * @file SentryCloudCan.hpp
  * @brief    哨兵云台CAN通信
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __PROJECT_SENTRY_CLOUD_     //定义工程标识符__PROJECT_SENTRY_CLOUD_
#define __PROJECT_SENTRY_CLOUD_
#endif // __PROJECT_SENTRY_CLOUD_

#include "can.h"
#include <string.h>
#include "bsp_can.hpp"
#include "sentry_can_commom.hpp"


#define CAN_INTERBOARD hcan2

void UpCloudCanCommuRoutine(void);  ///云台定时发送的板间CAN通信 - 上云台
void CloudCanFilterConfig(void);    ///云台用 设定CAN过滤器，以过滤不需要的ID号
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);  ///云台用 板间通讯CAN回调函数
// void ChassisCanRxHandle(void);


#endif // __SENTRY_CLOUD_CAN_COMMU_HPP_
