#ifndef __SENTRY_CLOUD_CAN_COMMU_HPP_
#define __SENTRY_CLOUD_CAN_COMMU_HPP_
/**
  * @brief    哨兵云台CAN通信
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "can.h"
#include "bsp_can.hpp"
#include "SentryCloudLogic.hpp"
#include <string.h>

enum SENTRY_CAN_ID  //板间通讯ID号
{
    UP_CLOUD_STATES = 0X101,
    DOWN_CLOUD_STATES = 0X102,
    CHASSIS_STATES = 0X103,
    SUPERIOR_UP_RELATIVE_CMD = 0X111,
    SUPERIOR_UP_ABSOLUTE_CMD = 0X112,
    SUPERIOR_DOWN_RELATIVE_CMD = 0X121,
    SUPERIOR_DOWN_ABSOLUTE_CMD = 0X122,
    SUPERIOR_CHASSIS_MOVE = 0X130,
    SUPERIOR_SAFE = 0x1A0
};

struct CanCommuRecv_t
{
    uint8_t SuperiorControlFlags;
    uint8_t UpCloudStates;
    uint8_t DownCloudStates;
    float SuperCon_Relative_Pitch;
    float SuperCon_Relative_Yaw;
    float SuperCon_Absolute_Pitch;
    float SuperCon_Absolute_yaw;
    float ChassisSpeed;
    float ChassisLocation;
};

extern CanCommuRecv_t CloudCanRecv;

HAL_StatusTypeDef CloudCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,uint8_t*  ptrData);
HAL_StatusTypeDef CloudCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,float argu1,float argu2);
void CanRxCpltCallBack_CommuUpdata(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef* RxHead,uint8_t* Data);

#endif // __SENTRY_CLOUD_CAN_COMMU_HPP_
