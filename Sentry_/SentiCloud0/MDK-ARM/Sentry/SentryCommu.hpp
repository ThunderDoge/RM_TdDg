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

//外设定义

//CAN板间通信定义，如果修CAN口请修改此处宏定义
#define CAN_INTERBOARD hcan2

//此板子使用了视觉串口
#define USE_VISION

//这是云台
#define CLOUD_COMMU

//这是底盘
//#define CHASSIS_COMMU

#ifdef USE_VISION
#include "bsp_vision.hpp"
#endif

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
#ifdef USE_VISION
//视觉串口数据帧功能字
enum __bsp_vision_Functionwords
{
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
    REBOOTINT = 0X08,
};
#endif
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
#ifdef USE_VISION
//视觉传输数据解析结构体
struct Sentry_vision_data
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
    uint8_t chassis_mode;
    //云台数据
    float Yaw;          //Yaw轴角度
    float Pitch;        //Pitch轴角度
    uint8_t Cloud_mode; //云台模式
    uint8_t cloud_ctrl_mode;
    //射击数据
    uint8_t Shoot_mode; //射击模式
    float Shoot_speed;  //射击速度
    uint8_t Shoot_freq; //射击频率
    //数据标志
    uint32_t UpdateTime;
    //日志系统使用
    uint8_t Error_code = 0;          //错误代码
    int16_t CAN1_motorlist = 0xffff; //CAN1电机列表
    int16_t CAN2_motorlist = 0xffff; //CAN2电机列表
};
#endif
#ifdef CLOUD_COMMU

#ifndef __CLOUD_MODE_DEF
#define __CLOUD_MODE_DEF
enum _cloud_ctrl_mode:uint8_t
{
    absolute_cloud = 0x01,
    relative_cloud = 0x02,
    save_cloud = 0x00,
};
#endif

#endif

#ifndef __CHASSIS_MODE_DEF
#define __CHASSIS_MODE_DEF
enum _chassis_mode:uint8_t
{
    _chassis_speed =1,
    _chassis_location =2,
    _chassis_location_limit_speed =3,
    _chassis_save =0,
};
#endif	//__CHASSIS_MODE_DEF

void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData);


//全局接收变量
extern CanCommuRecv_t CanRx, CanTx;
#ifdef USE_VISION
extern Sentry_vision_data VisionRx,VisionTx;
#endif
// 主任务中周期性发送函数
#ifdef CLOUD_COMMU
void CloudCanCommuRoutine(void);
#endif
#ifdef CHASSIS_COMMU
void ChassisCanCommuRoutine(void);
#endif
#ifdef USE_VISION
void CloudVisonTxRoutine(void);
#endif
//CAN接收回调自动接收函数
#ifdef CLOUD_COMMU
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);
#endif
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);
//CAN信息底盘托管控制程序
void ChassisCanRxHandle(void);
#ifdef USE_VISION
//视觉串口中断接收函数
void SentryVisionUartRxAll(uint8_t* Vision_Rxbuffer);

//视觉信息控制托管程序
void VisionRxHandle(void);
#endif

//废案。
//void UP_CLOUD_STATES_CanRx(int StdId, uint8_t *ptrData);
//void UP_CLOUD_STATES_CanTx();
//void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
//void DOWN_CLOUD_STATES_CanTx();
//void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
//void CHASSIS_STATES_CanTx();
//void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void UP_FEED_CanTx();
//void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void DOWN_FEED_CanTx();
//void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_UP_RELATIVE_CMD_CanTx();
//void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_UP_ABSOLUTE_CMD_CanTx();
//void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_DOWN_RELATIVE_CMD_CanTx();
//void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx();
//void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_MOVE_CanTx();
//void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
//void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx();
//void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_SAFE_CanTx();



#endif // __SENTRY_COMMU_HPP_
