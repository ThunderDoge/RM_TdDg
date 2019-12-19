/**
  * @file  SentryCommu.cpp
  * @brief    哨兵视觉、CAN通讯标识符/函数汇总
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/18
  * @version  v1.0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __SENTRY_COMMU_HPP_
#define __SENTRY_COMMU_HPP_

//外设宏定义，如果修CAN口请修改此处宏定义
#define CAN_INTERBOARD hcan2

#include "bsp_vision.hpp"
#include "bsp_can.hpp"
#include "SentryCanCommu.hpp"
enum SENTRY_CAN_ID : uint32_t //板间通讯ID号
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

//视觉串口数据帧功能字
enum __bsp_vision_Functionwords
{
    //帧头帧尾
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
    //视觉发给电控的
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //控制云台相对角度
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //控制云台绝对角度
    CMD_SHOOT = 0x03,                        //射击指令
    CMD_CHASSIS_CONTROL = 0X04,              //底盘控制
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //底盘控制路程
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //底盘控制路程带限速

    //电控发给视觉的
    CMD_GET_MCU_STATE = 0x11, //获取电控控制信息
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,
};
//ROBOT_ERR 的错误码列表
enum __bsp_vision_RobotError
{
    DBUS_OFFLINE = 0X01,
    CAN1_OFFLINE = 0X02,
    CAN2_OFFLINE = 0X03,
    MOTOR_OFFLINE_CNT = 0X04,
    GIMBOL_OFFLINE = 0X05,
    CHASSIS_OFFLINE = 0X06,
    JUDG_OFFLINE = 0X07,
    REBOOTINT = 0X08
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
    uint32_t RecvUpdateTime;
    //底盘信息
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    float Pillar_Dist;
    uint8_t Pillar_flag;
};

//视觉传输数据解析结构体
struct bsp_vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; //数据帧功能字
    //底盘数据
    float Vx;         //底盘X轴速度
    float Vy;         //底盘Y轴速度
    float Px;         //底盘X轴路程
    float Py;         //底盘Y轴路程
    float SpeedLimit; //底盘限速
    uint8_t pillar_flag;
    //云台数据
    float Yaw;          //Yaw轴角度
    float Pitch;        //Pitch轴角度
    uint8_t Cloud_mode; //云台模式
    //射击数据
    uint8_t Shoot_mode; //射击模式
    float Shoot_speed;  //射击速度
    uint8_t Shoot_freq; //射击频率
    //数据标志
    uint8_t Shoot_flag = 0; //视觉射击标志位
    uint8_t Ready_flag = 0; //数据就绪标志位

    //日志系统使用
    uint8_t Error_code = 0;          //错误代码
    int16_t CAN1_motorlist = 0xffff; //CAN1电机列表
    int16_t CAN2_motorlist = 0xffff; //CAN2电机列表
};

extern CanCommuRecv_t CanInfo;
extern bsp_vision_data VisionInfo;

#endif // __SENTRY_COMMU_HPP_
