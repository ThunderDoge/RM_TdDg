/**
  * @file  sentry_can_commom.hpp
  * @brief    CAN通讯公共信息
  * @details  
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */    
#ifndef __SENTRY_CAN_COMMOM_HPP_
#define __SENTRY_CAN_COMMOM_HPP_

#include "stm32f4xx.h"

enum SENTRY_CAN_ID : uint32_t //板间通讯ID号
{
    UP_CLOUD_STATES                             = 0X101U,   ///< 上云台状态信息
    DOWN_CLOUD_STATES                           = 0X102U,   ///< 下云台状态信息
    CHASSIS_STATES                              = 0X103U,   ///< 底盘状态信息
    CHASSIS_PILLAR                              = 0X104U,   ///< 底盘撞柱信息（之后会改掉）
    SUPERIOR_UP_RELATIVE_CMD                    = 0X111U,   ///< 
    SUPERIOR_UP_ABSOLUTE_CMD                    = 0X112U,
	SUPERIOR_UP_SPEED_CMD                       = 0X113U,
    SUPERIOR_DOWN_RELATIVE_CMD                  = 0X121U,
    SUPERIOR_DOWN_ABSOLUTE_CMD                  = 0X122U,
	SUPERIOR_DOWN_SPEED_CMD                     = 0X123U,
    SUPERIOR_CHASSIS_MOVE                       = 0X130U,
    SUPERIOR_CHASSIS_SET_LOACTION               = 0X131U,
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED   = 0X132U,
    SUPERIOR_SAFE                               = 0x1A0U,
};

struct CanCommuRecv_t
{
    uint32_t RecvId;
    uint8_t SuperiorControlFlags;
    float UpCloudPitchYaw[2];
    float DownCloudPitchYaw[2];
    float SuperCon_Relative_PitchYaw[2];
    float SuperCon_Absolute_PitchYaw[2];
    float Chassis_SpeedLocation[2];
    float Chassis_SpeedLimit;
    uint8_t feed_flag;
    uint32_t RecvUpdateTime;
    //底盘信息
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    float Pillar_Dist;
    uint8_t Pillar_flag;
};

HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2);

#endif // __SENTRY_CAN_COMMOM_HPP_
