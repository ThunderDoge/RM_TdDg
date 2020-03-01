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


#include "bsp_motor.hpp"
#include "app_imu.h"
#include "app_AmmoFeed.hpp"
//#include "app_vision.hpp"
#include "sentry_ctrl_def.hpp"

#ifndef __CLOUD_MODE_DEF
#endif


class SentryCloud   //云台电机集合
{
public:
    //Initializer & Destructor 构造与删除函数
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
    void Feed_Free_Fire_Set(int32_t FreeSpeed);
    void Feed_Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig);
    void Feed_Free_Once_Set(int32_t	DiscreDelay,int16_t trig);
    void Feed_Safe_Set();



    //陀螺仪数据经过旋转计算，储存在这里。周期性更新。
    float RotatedImuAngle[3];   //Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   //Roll,Pitch,Yaw
    float BaseImuAngleRate[3];

    //Public 状态
    int Mode;   ///<云台状态指示
    uint8_t err_flags=0;    ///<错误标志位。每一位的定义见：
	int shoot_flag=0;   ///<“正在射击”指示位
    int32_t Shoot_Speed=7000;   ///<摩擦轮的设定速度。
    float RealYaw;  ///< Yaw软角度
	float MechanicYaw;  ///< Yaw机械角度
    float RealPitch;    ///< Pitch软角度
    float TargetYaw;    ///< 受控制的时候设定的角度， DEBUG用
    float TargetPitch;  ///< 受控制的时候设定的角度， DEBUG用

    void Handle();  ///<云台自动控制函数
    void Safe_Set();    ///<安全模式
    void SetAngleTo(float pitch, float yaw);    ///<机械角度设定
    // void SetSoftAngleTo(float soft_pitch, float soft_yaw);  ///<软角度设定-未实现
    void SetAngleTo_Gyro(float pitch, float yaw);   ///<角度设定 陀螺仪控制模式

	void LazerSwitchCmd(int OnOrOff);   ///<开关激光灯
    void ShooterSwitchCmd(int OnOrOff); ///<开关射击许可位和摩擦轮
    float gravity_feedforward(float pitch){
        return g_A*cos(pitch+g_phi);
    }

private:
    static const float RotationMatrix[3][3];    ///<旋转矩阵陀螺仪到云台枪口方向。现在没用 
    //基本状态
    uint8_t shoot_is_permitted=0; ///<“允许射击”指示位。只能通过ShooterSwitchCmd开启。为0时不允许使用摩擦轮和拨弹电机。
    //PITCH重力补偿
    float g_A=0;  //重力补偿之系数
    float g_phi=0;    //重力补偿初相
    //裁判系统相关
    int RobotHP;    //现在的HP
    //视觉小主机通讯相关
    //板间CAN通讯相关
};

extern SentryCloud CloudEntity; ///非常重要

void pidPitchCallBack(pid* self);

#endif // __SENTRY_CLOUD_HPP_
