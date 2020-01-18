/**
 * @brief    哨兵云台电机控制集合 Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#ifndef __SENTRY_CLOUD_HPP_
#define __SENTRY_CLOUD_HPP_


//#include "bsp_motor.hpp"
//#include "bsp_vision.hpp"
#include "app_imu.h"
#include "app_AmmoFeed.hpp"

#ifndef __CLOUD_MODE_DEF
#define __CLOUD_MODE_DEF
enum _cloud_ctrl_mode:uint8_t
{
    absolute_cloud = 0x01,
    relative_cloud = 0x02,
    save_cloud = 0x00,
    absolute_gyro_cloud = 0x03,
	relative_gyro_cloud = 0x04,
	speed_cloud = 0x05,
};
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
	pid PitchSpeed;
    pid PitchPosition;
    pid PitchGyroPosition;
    pid PitchGyroSpeed;

    pid YawSpeed;
    pid YawPosition;
    pid YawGyroSpeed;
    pid YawGyroPosition;

    pid FricLeftSpeed;
    pid FricRightSpeed;

    pid FeedSpeed;
    pid FeedPositon;

    softcloud YawMotor;
    softcloud PitchMotor;

    motor FricLeftMotor;
    motor FricRightMotor;
    AmmoFeed Feed2nd;
    //陀螺仪数据经过旋转计算，储存在这里。周期性更新。
    float RotatedImuAngle[3];   //Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   //Roll,Pitch,Yaw
    float BaseImuAngleRate[3];

    //Public 状态
    int Mode;
	int shoot_flag=0;
    float RealYaw;
	float MechanicYaw;
    float RealPitch;
    float TargetYaw;
    float TargetPitch;

    void Handle();  //更新函数
    void Safe_Set();    //安全模式
    void SetAngleTo(float pitch, float yaw);    //机械角度设定
    // void SetSoftAngleTo(float soft_pitch, float soft_yaw);  //软角度设定-未实现
    void SetAngleTo_Gyro(float pitch, float yaw);
	void LazerSwitchCmd(int OnOrOff);

private:
    static const float RotationMatrix[3][3];
    //基本状态

    //裁判系统相关
    int RobotHP;
    //视觉小主机通讯相关
    //板间CAN通讯相关
};

extern SentryCloud Self;

void ModeSelect(void);

#endif // __SENTRY_CLOUD_HPP_
