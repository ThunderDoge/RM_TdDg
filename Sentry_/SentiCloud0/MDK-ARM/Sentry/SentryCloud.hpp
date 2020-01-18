/**
 * @brief    �ڱ���̨������Ƽ��� Sentry Cloud Motors Control
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


class SentryCloud   //��̨�������
{
public:
    //Initializer & Destructor ������ɾ������
    SentryCloud(uint8_t yaw_can_num,uint16_t yaw_can_id, 
    uint8_t pitch_can_num, uint16_t pitch_can_id, 
    uint8_t fric_l_can_num, uint16_t fric_l_can_id,
    uint8_t fric_r_can_num,uint16_t fric_r_can_id,
    uint8_t feed_can_num,uint16_t feed_can_id);
    //Method to Handle ���Ʒ���
	//���棡WARNING!��Ա��������������˳���ʼ��
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
    //���������ݾ�����ת���㣬��������������Ը��¡�
    float RotatedImuAngle[3];   //Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   //Roll,Pitch,Yaw
    float BaseImuAngleRate[3];

    //Public ״̬
    int Mode;
	int shoot_flag=0;
    float RealYaw;
	float MechanicYaw;
    float RealPitch;
    float TargetYaw;
    float TargetPitch;

    void Handle();  //���º���
    void Safe_Set();    //��ȫģʽ
    void SetAngleTo(float pitch, float yaw);    //��е�Ƕ��趨
    // void SetSoftAngleTo(float soft_pitch, float soft_yaw);  //��Ƕ��趨-δʵ��
    void SetAngleTo_Gyro(float pitch, float yaw);
	void LazerSwitchCmd(int OnOrOff);

private:
    static const float RotationMatrix[3][3];
    //����״̬

    //����ϵͳ���
    int RobotHP;
    //�Ӿ�С����ͨѶ���
    //���CANͨѶ���
};

extern SentryCloud Self;

void ModeSelect(void);

#endif // __SENTRY_CLOUD_HPP_
