#ifndef __SENTRY_CLOUD_CAN_COMMU_HPP_
#define __SENTRY_CLOUD_CAN_COMMU_HPP_


/** 
 * @file SentryCloudCan.hpp
  * @brief    �ڱ���̨CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __PROJECT_SENTRY_DOWN_CLOUD_     //���幤�̱�ʶ��__PROJECT_SENTRY_DOWN_CLOUD_
#define __PROJECT_SENTRY_DOWN_CLOUD_
#endif // __PROJECT_SENTRY_DOWN_CLOUD_

//CAN���ͨ�Ŷ��壬�����CAN�����޸Ĵ˴��궨��
#define CAN_INTERBOARD hcan2

#include "can.h"
#include <string.h>
#include "bsp_can.hpp"
#include "sentry_can_commom.hpp"



void UpCloudCanCommuRoutine(void);  ///��̨��ʱ���͵İ��CANͨ�� - ����̨
void CloudCanFilterConfig(void);    ///��̨�� �趨CAN���������Թ��˲���Ҫ��ID��
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);  ///��̨�� ���ͨѶCAN�ص�����
// void ChassisCanRxHandle(void);


#endif // __SENTRY_CLOUD_CAN_COMMU_HPP_
