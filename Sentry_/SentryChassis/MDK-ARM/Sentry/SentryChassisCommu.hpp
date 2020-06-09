/**
  * @file   SentryChassisCommu.cpp
  * @brief      哨兵底盘通信
  * @details  
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CHASSIS_COMMU_
#define __SENTRY_CHASSIS_COMMU_

//依赖的文件
#include "SentryChassisCan.hpp"
// #include "SentryChassisUart.hpp"

extern CanCommuRecv_t CanRx, CanTx;

void ChassisCanCommuRoutine(void);  ///底盘定时发送的板间CAN通信

#endif // __SENTRY_CHASSIS_COMMU_
