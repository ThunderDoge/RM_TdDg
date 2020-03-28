/**
 * @file SentryCloud.hpp
 * @brief    哨兵云台电机控制集合 Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#ifndef __SENTRY_CLOUD_HPP_
#define __SENTRY_CLOUD_HPP_

// 依赖的文件
#include "bsp_motor.hpp"
#include "app_imu.h"
#include "app_AmmoFeed.hpp"
//#include "app_vision.hpp"
#include "sentry_ctrl_def.hpp"
#include "app_mode.hpp"
#include "app_sentry_check_device.hpp"




/**
 * @brief 云台物理实体类，包含 操纵云台进行物理运动的函数 和 物理信息，以及相关配置信息
 * 
 */
class SentryCloud   
{
public:
    //Initializer & Destructor 云台物理实体类 构造与删除函数
    SentryCloud(uint8_t yaw_can_num,uint16_t yaw_can_id, 
    uint8_t pitch_can_num, uint16_t pitch_can_id, 
    uint8_t fric_l_can_num, uint16_t fric_l_can_id,
    uint8_t fric_r_can_num,uint16_t fric_r_can_id,
    uint8_t feed_can_num,uint16_t feed_can_id);
    //Method to Handle 控制方法
	//警告！WARNING!成员变量将以声明的顺序初始化
	//warning:  #1299-D: members and base-classes will be initialized in declaration order, not in member initialisation list order
	pid PitchSpeed;         ///<Pitch电机机械角 速度环
    pid PitchPosition;      ///<Pitch电机机械角 位置环
    pid PitchGyroPosition;  ///<Pitch电机陀螺仪 位置环
    pid PitchGyroSpeed;     ///<Pitch电机陀螺仪 速度环

    pid YawSpeed;           ///<Yaw电机机械角 速度环
    pid YawPosition;        ///<Yaw电机机械角 位置环
    pid YawGyroSpeed;       ///<Yaw电机陀螺仪 速度环
    pid YawGyroPosition;    ///<Yaw电机陀螺仪 位置环

    pid FricLeftSpeed;      ///<左边摩擦轮 速度环
    pid FricRightSpeed;     ///<右边摩擦轮 速度环

    pid FeedSpeed;          ///<供弹轮 速度环
    pid FeedPositon;        ///<供弹轮 位置环

    softcloud YawMotor;     ///<Yaw轴电机对象_CAN1上
    softcloud PitchMotor;   ///<Pitch轴电机对象_CAN1上

    motor FricLeftMotor;    ///< 左边摩擦轮
    motor FricRightMotor;   ///< 右边摩擦轮
    AmmoFeed Feed2nd;       ///< 供弹轮电机
public:
    //为了安全，提供了包装供弹轮函数，需要确认shoot_is_permitted才能运行供弹轮。提不要直接使用Feed2nd
    void Feed_Free_Fire_Set(int32_t FreeSpeed);                 ///< 供弹轮：连续转动设置
    void Feed_Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig);  ///< 供弹轮：n连发设置
    void Feed_Free_Once_Set(int32_t	DiscreDelay,int16_t trig);  ///< 供弹轮：单发设置
    void Feed_Safe_Set();   ///< 供弹轮停止



    //陀螺仪数据经过旋转计算，储存在这里。周期性更新。
    float RotatedImuAngle[3];   ///<云台枪口朝向：Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   ///<云台枪口朝向角速度Roll,Pitch,Yaw
    float BaseImuAngleRate[3];      ///< 云台水平底座朝向

    //Public 状态
    int Mode;   ///<云台状态指示
    uint8_t force_use_mech_gyro=0;
    uint8_t err_flags=0;    ///<错误标志位。每一位的定义见：
	int shoot_flag=0;   ///<“正在射击”指示位
    int32_t Shoot_Speed=7000;   ///<摩擦轮的设定速度。
    float RealYaw;  ///< Yaw软角度
	float MechanicYaw;  ///< Yaw机械角度
    float RealPitch;    ///< Pitch软角度
    float TargetYaw;    ///< 受控制的时候设定的角度， DEBUG用
    float TargetPitch;  ///< 受控制的时候设定的角度， DEBUG用

    void Handle();  ///< 云台自动控制函数，包含着所有数据的获取、处理和例行执行。应当在主逻辑任务中调用。>>>>>>>>>>>>>>>>>重要<<<<<<<<<<<<<<
    void Safe_Set();    ///<安全模式
    void SetAngleTo(float pitch, float yaw);    ///<机械角度设定
//    void SetSoftAngleTo(float soft_pitch, float soft_yaw);  ///<软角度设定-未实现
//    void SetCtrlMode_Force(enum _cloud_ctrl_mode);  ///< 强制设定控制模式
    void SetAngleTo_Gyro(float pitch, float yaw);   ///<角度设定 陀螺仪控制模式

	void LazerSwitchCmd(int OnOrOff);   ///<开关激光灯
    void ShooterSwitchCmd(int OnOrOff); ///<开关射击许可位和摩擦轮
    float gravity_feedforward(float pitch){ ///< 重力前馈补偿函数，内部使用
        return g_A*cos(pitch+g_phi);
    }

private:
    static const float RotationMatrix[3][3];    ///<旋转矩阵陀螺仪到云台枪口方向。现在没用 
    //基本状态
    uint8_t shoot_is_permitted=0; ///<“允许射击”指示位。只能通过ShooterSwitchCmd开启。为0时不允许使用摩擦轮和拨弹电机。
    uint8_t forced_ctrl_mode = (uint8_t)auto_cloud; // 强行指定的控制模式
    //PITCH重力补偿
    float g_A=0;  //重力补偿之系数
    float g_phi=0;    //重力补偿初相
    //裁判系统相关
    int RobotHP;    //现在的HP
    //视觉小主机通讯相关
    //板间CAN通讯相关
};

extern SentryCloud CloudEntity; ///云台物理实体对象。包含电机激光器等设备。

// 设备对象。离线检测用
extern CheckDevice_Type UpCloudLeftFric_CheckDevice;
extern CheckDevice_Type UpCloudRightFric_CheckDevice;
extern CheckDevice_Type UpCloudYawMotor_CheckDevice;
extern CheckDevice_Type UpCloudPitchMotor_CheckDevice;
extern CheckDevice_Type UpCloudFeedMotor_CheckDevice;

// 云台控制模式相关
//void EnterModeCloudCtrlMech(void);
//void RunModeCloudCtrlMech(void);
//void EnterModeCloudCtrlGyro(void);
//void RunModeCloudCtrlGyro(void);

//extern Mode ModeCloudCtrlMech;  // 机械角位置环，陀螺仪速度环
//extern Mode ModeCloudCtrlGyro;  // 陀螺仪 位置环&速度环

/// PID运算回调。用于PITCH重力前馈 逻辑
void pidPitchCallBack(pid* self);

#endif // __SENTRY_CLOUD_HPP_
