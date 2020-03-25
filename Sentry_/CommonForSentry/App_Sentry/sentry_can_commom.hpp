/**
 * @file  sentry_can_commom.hpp
 * @brief    CAN通讯公共信息
 * @details
 * 哨兵CAN相关的函数全部储存在此文件之内，便于修改协议时统一修改。云台和底盘都依赖于此文件。
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
#include "string.h"
#include "stm32f4xx_hal.h"
#include "bsp_can.hpp"         //需要使用CAN
#include "sentry_config.hpp"   //包含了工程标识符 __PROJECT_SENTRY_CLOUD_ 或者 __PROJECT_SENTRY_CHASSIS_
#include "sentry_ctrl_def.hpp" //依赖于MCU模式枚举值

#ifdef __PROJECT_SENTRY_CLOUD_ //根据工程标识符加载对应的依赖文件
#include "SentryCloud.hpp"
#endif // __PROJECT_SENTRY_CLOUD_

#ifdef __PROJECT_SENTRY_CHASSIS_ //根据工程标识符加载对应   的依赖文件
#include "SentryChassis.hpp"
#endif //__PROJECT_SENTRY_CHASSIS_

//必须定义的宏定义。如果未定义会帮你定义
#ifndef CAN_INTERBOARD
#define CAN_INTERBOARD hcan2
#endif // CAN_INTERBOARD

#define SENTRY_CAN_ID_CNT 20

/**
 * 2020-2-27 ThunderDoge观察到，绝大多数的
 *
 *
 *
 * 程序是如何接收CAN消息的
 * 首先CAN的接收邮箱已经经过设置，使得它仅接收自己需要的内容的CANID
 * 然后，一个回调函数会在Can回调函数中被调用，它接收CAN已经收到的StdId和数据指针pData。他将顺序遍历一个哨兵
 * 通信用CANID列表[ SENTRY_CAN_ID_List
 * ]。如果有一个符合，他将按照数据指令列表[]中的序号相同位置的指令(指令如枚举[
 * __enum_sentry_can_id_RW_OrderWord ]所示)，储存到数据目标列表[]所示的地址中。
 * 所以每当你需要通信功能。你需要：
 * 1. 添加 SENTRY_CAN_ID 条目
 * 2. 添加 CANID列表[] 条目
 * 3. 添加 通讯指令列表条目
 * 4. 添加 存取地址列表条目
 */
enum SENTRY_CAN_ID : uint32_t //板间通讯ID号
{
    //通用接收 0x10X
    UP_CLOUD_STATES = 0X101U,                           ///< 上云台状态信息
    DOWN_CLOUD_STATES = 0X102U,                         ///< 下云台状态信息
    CHASSIS_STATES = 0X103U,                            ///< 底盘状态信息
    CHASSIS_PILLAR = 0X104U,                            ///< 底盘撞柱信息（之后会改掉）
    OFFLINE_LIST =0X105U,                               ///< 离线设备列表
    SUPERIOR_SAFE = 0x106U,                             ///< 底盘安全

    //上云台接收 0x11X
    SUPERIOR_UP_RELATIVE_CMD = 0X111U,                  ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_UP_ABSOLUTE_CMD = 0X112U,                  ///< SUPERCMD上云台绝对角度控制
    SUPERIOR_UP_SPEED_CMD = 0X113U,                     ///< 【未启用】SUPERCMD上云台转动速度控制
    //下云台接收 0x12X
    SUPERIOR_DOWN_RELATIVE_CMD = 0X121U,                ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_DOWN_ABSOLUTE_CMD = 0X122U,                ///< SUPERCMD上云台绝对角度控制
    SUPERIOR_DOWN_SPEED_CMD = 0X123U,                   ///< 【未启用】SUPERCMD上云台转动速度控制
    //底盘接收 0x13x
    SUPERIOR_CHASSIS_MOVE = 0X130U,                     ///< SUPERCMD底盘速度控制
    SUPERIOR_CHASSIS_SET_LOACTION = 0X131U,             ///< 底盘位置控制
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED = 0X132U, ///< 底盘位置控制加限速

};

// CanRx,CanTx内时间戳的数量
#define __SENTRY_CAN_UPDATE_TIMESTAMP_COUNT 20 // CAN缓存机构有多少个时间戳

enum enumSentryCanUpdataTimestampPosition : int
{ // CAN缓存的时间戳编号枚举
    tCanRecv = 0,
    tSuperiorControl,
    tSuperCon_UpCloud,
    tSuperCon_DownCloud,
    tSuperCon_Chassis,

    tUpCloud_Info,
    tUpCloud_Shoot_Info,
    tDownCloud_Info,
    tDownCloud_Shoot_Info,
    tChassis_Info,
    tChassis_Pil_Info,
};
struct CanCommuRecv_t /// CAN缓存结构体定义
{
    //通用信息
    uint32_t RecvId;
    uint32_t CanUpdateTime[__SENTRY_CAN_UPDATE_TIMESTAMP_COUNT];
    //上级指令
    // uint8_t SuperiorControlFlags;
    uint8_t SuperCon_CloudMode;
    uint8_t SuperCon_CloudFireOrder : 1;
    float SuperCon_PitchYaw[2];
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    //云台信息
    float UpCloudPitchYaw[2];
    float DownCloudPitchYaw[2];
    uint8_t Shooting_Flag;
    //底盘信息
    float Chassis_SpeedLocation[2];
    float Chassis_SpeedLimit;
    float Pillar_Dist;
    uint8_t Pillar_flag;
};
#undef __SENTRY_CAN_UPDATE_TIMESTAMP_COUNT //记得取消局部用DEFINE

extern CanCommuRecv_t CanRx, CanTx; //声明CAN缓存结构体

extern uint32_t
    SENTRY_CAN_ID_List[SENTRY_CAN_ID_CNT]; /// SENTRY_CAN_ID 待匹配列表

extern uint8_t SENTRY_CAN_ID_RwOrder_List[SENTRY_CAN_ID_CNT];

extern void *SENTRY_CAN_ID_RwPosition_List[SENTRY_CAN_ID_CNT][2];

extern uint8_t IS_SUPERIOR_VISION_CTRL, IS_SUPERIOR_MANUAL_CTRL;

// HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
//                                 uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
                                float argu1, float argu2);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
                                uint8_t *ptrData, size_t size=8);

/**
 * @addtogroup CAN_Interboard_Communication
 * @{
 */
/**
 * @addtogroup CAN_Interboard_COmmu_StatusBroudcast
 * @{
 */
void ChassisCanCommuRoutine(void);   ///底盘定时发送的板间CAN通信
void UpCloudCanCommuRoutine(void);   ///云台定时发送的板间CAN通信 - 上云台
void DownCloudCanCommuRoutine(void); ///云台定时发送的板间CAN通信 - 下云台

void UP_CLOUD_STATES_CanRx(
    int StdId,
    uint8_t *
        ptrData);                                               ///从 CAN_INTERBOARD
                                                                ///接收上云台数据帧(UPCLOUD_STATES)数据到CanRx。如果功能字不对则忽略。
void UP_CLOUD_STATES_CanTx();                                   ///从 CanTx 的数据通过 CAN_INTERBOARD
                                                                ///发送上云台数据帧(UPCLOUD_STATES)
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
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId,
                                                     uint8_t *ptrData);
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
