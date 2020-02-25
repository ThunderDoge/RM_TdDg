/**
  * @file  sentry_can_commom.hpp
  * @brief    CAN通讯公共信息
  * @details  哨兵CAN相关的函数全部储存在此文件之内，便于修改协议时统一修改。云台和底盘都依赖于此文件。
  * 但是并不是云台和底盘都需要此文件中的的全部函数。
  * 所以云台和底盘分别具有宏定义：
  * __PROJECT_SENTRY_CLOUD_
  * __PROJECT_SENTRY_CHASSIS_
  * 本文件使用条件编译分别给予他们各自需要的函数。
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */    
#ifndef __SENTRY_CAN_COMMOM_HPP_
#define __SENTRY_CAN_COMMOM_HPP_

//依赖的文件
#include "stm32f4xx_hal.h"
#include "string.h"
#include "bsp_can.hpp"          //需要使用CAN
#include "sentry_ctrl_def.hpp"  //依赖于MCU模式枚举值
#include "sentry_config.hpp"    //包含了工程标识符 __PROJECT_SENTRY_CLOUD_ 或者 __PROJECT_SENTRY_CHASSIS_


#ifdef __PROJECT_SENTRY_CLOUD_  //根据工程标识符加载对应的依赖文件
#include "SentryCloud.hpp"
#endif // __PROJECT_SENTRY_CLOUD_

#ifdef __PROJECT_SENTRY_CHASSIS_    //根据工程标识符加载对应的依赖文件
#include "SentryChassis.hpp"
#endif //__PROJECT_SENTRY_CHASSIS_

//必须定义的宏定义。如果未定义会帮你定义
#ifndef CAN_INTERBOARD
#define CAN_INTERBOARD hcan2
#endif // CAN_INTERBOARD

enum SENTRY_CAN_ID : uint32_t //板间通讯ID号
{
    UP_CLOUD_STATES                             = 0X101U,   ///< 上云台状态信息
    DOWN_CLOUD_STATES                           = 0X102U,   ///< 下云台状态信息
    CHASSIS_STATES                              = 0X103U,   ///< 底盘状态信息
    CHASSIS_PILLAR                              = 0X104U,   ///< 底盘撞柱信息（之后会改掉）
    SUPERIOR_UP_RELATIVE_CMD                    = 0X111U,   ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_UP_ABSOLUTE_CMD                    = 0X112U,   ///< SUPERCMD上云台绝对角度控制
	SUPERIOR_UP_SPEED_CMD                       = 0X113U,   ///< 【未启用】SUPERCMD上云台转动速度控制
    SUPERIOR_DOWN_RELATIVE_CMD                  = 0X121U,   ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_DOWN_ABSOLUTE_CMD                  = 0X122U,   ///< SUPERCMD上云台绝对角度控制
	SUPERIOR_DOWN_SPEED_CMD                     = 0X123U,   ///< 【未启用】SUPERCMD上云台转动速度控制
    SUPERIOR_CHASSIS_MOVE                       = 0X130U,   ///< SUPERCMD底盘速度控制
    SUPERIOR_CHASSIS_SET_LOACTION               = 0X131U,   ///< 底盘位置控制
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED   = 0X132U,   ///< 底盘位置控制加限速
    SUPERIOR_SAFE                               = 0x1A0U,   ///< 底盘安全
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

extern CanCommuRecv_t CanRx,CanTx;

extern uint8_t IS_SUPERIOR_VISION_CTRL,IS_SUPERIOR_MANUAL_CTRL;

HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2);

/**
 * @addtogroup CAN_Interboard_Communication
 * @{
 */
/**
 * @addtogroup CAN_Interboard_COmmu_StatusBroudcast
 * @{
 */
void ChassisCanCommuRoutine(void);  ///底盘定时发送的板间CAN通信
void UpCloudCanCommuRoutine(void);  ///云台定时发送的板间CAN通信 - 上云台
void DownCloudCanCommuRoutine(void);    ///云台定时发送的板间CAN通信 - 下云台

void UP_CLOUD_STATES_CanRx(int StdId, uint8_t *ptrData);    ///从 CAN_INTERBOARD 接收上云台数据帧(UPCLOUD_STATES)数据到CanRx。如果功能字不对则忽略。
void UP_CLOUD_STATES_CanTx();                               ///从 CanTx 的数据通过 CAN_INTERBOARD 发送上云台数据帧(UPCLOUD_STATES)
void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData); ///以下类推。
void DOWN_CLOUD_STATES_CanTx();
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
void CHASSIS_STATES_CanTx();
/**
 * @}
 * CAN_Interboard_COmmu_StatusBroudcast
 */
/**
 * @addtogroup CAN_Interboard_Commu_SuperiorCommand
 * @{
 */
void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData);
void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
void UP_FEED_CanTx();
void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
void DOWN_FEED_CanTx();
void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_UP_RELATIVE_CMD_CanTx();
void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_UP_ABSOLUTE_CMD_CanTx();
void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_DOWN_RELATIVE_CMD_CanTx();
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx();
void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_CHASSIS_MOVE_CanTx();
void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx();
void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_SAFE_CanTx();
/**
 * @}
 * CAN_Interboard_Commu_SuperiorCommand
 */
/**
 * @}
 * CAN_Interboard_Communication
 */

#endif // __SENTRY_CAN_COMMOM_HPP_
