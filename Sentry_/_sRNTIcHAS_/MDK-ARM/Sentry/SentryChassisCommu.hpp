/**
  * @file   SentryChassisCommu.cpp
  * @brief      �ڱ�����ͨ��
  * @details  
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CHASSIS_COMMU_
#define __SENTRY_CHASSIS_COMMU_

//�������ļ�
#include "SentryChassisCan.hpp"
#include "SentryChassisUart.hpp"

extern CanCommuRecv_t CanRx, CanTx;

void ChassisCanCommuRoutine(void);  ///���̶�ʱ���͵İ��CANͨ��

#endif // __SENTRY_CHASSIS_COMMU_
