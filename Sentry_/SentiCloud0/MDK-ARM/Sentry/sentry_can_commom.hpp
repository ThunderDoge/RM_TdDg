/**
  * @file  sentry_can_commom.hpp
  * @brief    CANͨѶ������Ϣ
  * @details  
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */    
#ifndef __SENTRY_CAN_COMMOM_HPP_
#define __SENTRY_CAN_COMMOM_HPP_

#include "stm32f4xx.h"

enum SENTRY_CAN_ID : uint32_t //���ͨѶID��
{
    UP_CLOUD_STATES                             = 0X101U,   ///< ����̨״̬��Ϣ
    DOWN_CLOUD_STATES                           = 0X102U,   ///< ����̨״̬��Ϣ
    CHASSIS_STATES                              = 0X103U,   ///< ����״̬��Ϣ
    CHASSIS_PILLAR                              = 0X104U,   ///< ����ײ����Ϣ��֮���ĵ���
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
    //������Ϣ
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    float Pillar_Dist;
    uint8_t Pillar_flag;
};

HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2);

#endif // __SENTRY_CAN_COMMOM_HPP_
