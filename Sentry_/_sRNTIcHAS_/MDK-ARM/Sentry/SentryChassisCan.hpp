/**
  * @file   SentryChassisCan.hpp
  * @brief      哨兵底盘CAN通信
  * @details    
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CHASSIS_CAN_HPP_
#define __SENTRY_CHASSIS_CAN_HPP_

//CAN板间通信定义，如果修CAN口请修改此处宏定义
#define CAN_INTERBOARD hcan2

//依赖的文件
#include "sentry_can_commom.hpp"
#include "sentry_config.hpp"


void ChassisCanCommuRoutine(void);  ///底盘定时发送的板间CAN通信
void ChassisCanFilterConfig(void);    ///云台用 设定CAN过滤器，以过滤不需要的ID号
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);  ///云台用 板间通讯CAN回调函数

#endif // __SENTRY_CHASSIS_CAN_HPP_
