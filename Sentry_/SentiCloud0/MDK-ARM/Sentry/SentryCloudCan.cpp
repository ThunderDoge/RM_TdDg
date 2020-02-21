//
/**
  * @file SentryCloudCan.cpp
  * @brief    �ڱ���̨CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCanCommu.hpp"
#include <cstring>
//ȫ��CAN���ձ���
/**
 * @brief ��̨�ã�MCU��CANͨѶ��CAN�ص�����
 * 
 * @param     _hcan ����
 * @param     RxHead 
 * @param     Data 
 */
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    // ����̨����Ҫ�����Լ�����Ϣ
    // UP_CLOUD_STATES_CanRx();
    DOWN_CLOUD_STATES_CanRx(RxHead->StdId, Data);
    CHASSIS_STATES_CanRx(RxHead->StdId, Data);
}
void CloudCanFilterConfig()
{
    
}

