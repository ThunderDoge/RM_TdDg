#ifndef __SENTRY_CLOUD_CAN_COMMU_HPP_
#define __SENTRY_CLOUD_CAN_COMMU_HPP_
/**
  * @brief    �ڱ���̨CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "can.h"
#include "bsp_can.hpp"
#include "SentryChassisLogic.hpp"
#include <string.h>

#define CAN_INTERBOARD hcan2

enum SENTRY_CAN_ID : uint32_t //���ͨѶID��
{
    UP_CLOUD_STATES = 0X101U,
    DOWN_CLOUD_STATES = 0X102U,
    CHASSIS_STATES = 0X103U,
	CHASSIS_PILLAR = 0X104U,
    SUPERIOR_UP_RELATIVE_CMD = 0X111U,
    SUPERIOR_UP_ABSOLUTE_CMD = 0X112U,
    SUPERIOR_DOWN_RELATIVE_CMD = 0X121U,
    SUPERIOR_DOWN_ABSOLUTE_CMD = 0X122U,
    UP_FEED = 0X113U,
    DOWN_FEED = 0X123U,
    SUPERIOR_CHASSIS_MOVE = 0X130U,
    SUPERIOR_CHASSIS_SET_LOACTION = 0X131U,
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED = 0X132U,
    SUPERIOR_SAFE = 0x1A0U,
};
enum SuperiorControlFlag_for_Chassis
{
    _SUPERIOR_CHASSIS_SPEED_SET_ = 0X01,
    _SUPERIOR_CHASSIS_LOACATION_SET_ = 0X02,
    _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_ = 0X03,
    _SUPERIOR_OFFLINE_ = 0,
};

struct CanCommuRecv_t
{
    uint32_t RecvId;
    uint8_t Ready_Flag;
    uint8_t SuperiorControlFlags;
    uint8_t UpCloudStates;
    uint8_t DownCloudStates;
    float UpCloudPitchYaw[2];
    float UpFeedSpeed;
    float DownFeedSpeed;
    float DownCloudPitchYaw[2];
    float SuperCon_Relative_Pitch;
    float SuperCon_Relative_Yaw;
    float SuperCon_Absolute_Pitch;
    float SuperCon_Absolute_yaw;
    float ChassisSpeed;
    float ChassisLocation;
    float ChassisSpeedLimit;
    uint32_t SuperiorUpdateTime;
    uint32_t RecvUpdateTime;
    //������Ϣ
    uint8_t chassis_control_mode;
    uint8_t pillar_close_flag;
    float location_on_rail;
};

extern CanCommuRecv_t CanRecv;

HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, float argu1, float argu2);
void CanRxCpltCallBack_CommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);
HAL_StatusTypeDef ChassisCanCommuRoutine(void);
#endif // __SENTRY_CLOUD_CAN_COMMU_HPP_