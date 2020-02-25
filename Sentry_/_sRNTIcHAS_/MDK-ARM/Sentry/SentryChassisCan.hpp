/**
  * @file   SentryChassisCan.hpp
  * @brief      �ڱ�����CANͨ��
  * @details    
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CHASSIS_CAN_HPP_
#define __SENTRY_CHASSIS_CAN_HPP_

//CAN���ͨ�Ŷ��壬�����CAN�����޸Ĵ˴��궨��
#define CAN_INTERBOARD hcan2

//�������ļ�
#include "sentry_can_commom.hpp"
#include "sentry_config.hpp"


void ChassisCanCommuRoutine(void);  ///���̶�ʱ���͵İ��CANͨ��
void ChassisCanFilterConfig(void);    ///��̨�� �趨CAN���������Թ��˲���Ҫ��ID��
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);  ///��̨�� ���ͨѶCAN�ص�����

#endif // __SENTRY_CHASSIS_CAN_HPP_
