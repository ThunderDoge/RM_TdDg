//
/**
  * @file SentryCloudCan.cpp
  * @brief    哨兵云台CAN通信
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCanCommu.hpp"
#include <cstring>
//全局CAN接收变量
/**
 * @brief 云台用，MCU间CAN通讯，CAN回调函数
 * 
 * @param     _hcan 套用
 * @param     RxHead 
 * @param     Data 
 */
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    // 上云台不需要接收自己的信息
    // UP_CLOUD_STATES_CanRx();
    DOWN_CLOUD_STATES_CanRx(RxHead->StdId, Data);
    CHASSIS_STATES_CanRx(RxHead->StdId, Data);
}
void CloudCanFilterConfig()
{
    
}

