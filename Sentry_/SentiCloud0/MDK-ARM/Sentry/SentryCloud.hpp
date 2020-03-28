/**
 * @file SentryCloud.hpp
 * @brief    �ڱ���̨������Ƽ��� Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#ifndef __SENTRY_CLOUD_HPP_
#define __SENTRY_CLOUD_HPP_

// �������ļ�
#include "bsp_motor.hpp"
#include "app_imu.h"
#include "app_AmmoFeed.hpp"
//#include "app_vision.hpp"
#include "sentry_ctrl_def.hpp"
#include "app_mode.hpp"
#include "app_sentry_check_device.hpp"




/**
 * @brief ��̨����ʵ���࣬���� ������̨���������˶��ĺ��� �� ������Ϣ���Լ����������Ϣ
 * 
 */
class SentryCloud   
{
public:
    //Initializer & Destructor ��̨����ʵ���� ������ɾ������
    SentryCloud(uint8_t yaw_can_num,uint16_t yaw_can_id, 
    uint8_t pitch_can_num, uint16_t pitch_can_id, 
    uint8_t fric_l_can_num, uint16_t fric_l_can_id,
    uint8_t fric_r_can_num,uint16_t fric_r_can_id,
    uint8_t feed_can_num,uint16_t feed_can_id);
    //Method to Handle ���Ʒ���
	//���棡WARNING!��Ա��������������˳���ʼ��
	//warning:  #1299-D: members and base-classes will be initialized in declaration order, not in member initialisation list order
	pid PitchSpeed;         ///<Pitch�����е�� �ٶȻ�
    pid PitchPosition;      ///<Pitch�����е�� λ�û�
    pid PitchGyroPosition;  ///<Pitch��������� λ�û�
    pid PitchGyroSpeed;     ///<Pitch��������� �ٶȻ�

    pid YawSpeed;           ///<Yaw�����е�� �ٶȻ�
    pid YawPosition;        ///<Yaw�����е�� λ�û�
    pid YawGyroSpeed;       ///<Yaw��������� �ٶȻ�
    pid YawGyroPosition;    ///<Yaw��������� λ�û�

    pid FricLeftSpeed;      ///<���Ħ���� �ٶȻ�
    pid FricRightSpeed;     ///<�ұ�Ħ���� �ٶȻ�

    pid FeedSpeed;          ///<������ �ٶȻ�
    pid FeedPositon;        ///<������ λ�û�

    softcloud YawMotor;     ///<Yaw��������_CAN1��
    softcloud PitchMotor;   ///<Pitch��������_CAN1��

    motor FricLeftMotor;    ///< ���Ħ����
    motor FricRightMotor;   ///< �ұ�Ħ����
    AmmoFeed Feed2nd;       ///< �����ֵ��
public:
    //Ϊ�˰�ȫ���ṩ�˰�װ�����ֺ�������Ҫȷ��shoot_is_permitted�������й����֡��᲻Ҫֱ��ʹ��Feed2nd
    void Feed_Free_Fire_Set(int32_t FreeSpeed);                 ///< �����֣�����ת������
    void Feed_Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig);  ///< �����֣�n��������
    void Feed_Free_Once_Set(int32_t	DiscreDelay,int16_t trig);  ///< �����֣���������
    void Feed_Safe_Set();   ///< ������ֹͣ



    //���������ݾ�����ת���㣬��������������Ը��¡�
    float RotatedImuAngle[3];   ///<��̨ǹ�ڳ���Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   ///<��̨ǹ�ڳ�����ٶ�Roll,Pitch,Yaw
    float BaseImuAngleRate[3];      ///< ��̨ˮƽ��������

    //Public ״̬
    int Mode;   ///<��̨״ָ̬ʾ
    uint8_t force_use_mech_gyro=0;
    uint8_t err_flags=0;    ///<�����־λ��ÿһλ�Ķ������
	int shoot_flag=0;   ///<�����������ָʾλ
    int32_t Shoot_Speed=7000;   ///<Ħ���ֵ��趨�ٶȡ�
    float RealYaw;  ///< Yaw��Ƕ�
	float MechanicYaw;  ///< Yaw��е�Ƕ�
    float RealPitch;    ///< Pitch��Ƕ�
    float TargetYaw;    ///< �ܿ��Ƶ�ʱ���趨�ĽǶȣ� DEBUG��
    float TargetPitch;  ///< �ܿ��Ƶ�ʱ���趨�ĽǶȣ� DEBUG��

    void Handle();  ///< ��̨�Զ����ƺ������������������ݵĻ�ȡ�����������ִ�С�Ӧ�������߼������е��á�>>>>>>>>>>>>>>>>>��Ҫ<<<<<<<<<<<<<<
    void Safe_Set();    ///<��ȫģʽ
    void SetAngleTo(float pitch, float yaw);    ///<��е�Ƕ��趨
//    void SetSoftAngleTo(float soft_pitch, float soft_yaw);  ///<��Ƕ��趨-δʵ��
//    void SetCtrlMode_Force(enum _cloud_ctrl_mode);  ///< ǿ���趨����ģʽ
    void SetAngleTo_Gyro(float pitch, float yaw);   ///<�Ƕ��趨 �����ǿ���ģʽ

	void LazerSwitchCmd(int OnOrOff);   ///<���ؼ����
    void ShooterSwitchCmd(int OnOrOff); ///<����������λ��Ħ����
    float gravity_feedforward(float pitch){ ///< ����ǰ�������������ڲ�ʹ��
        return g_A*cos(pitch+g_phi);
    }

private:
    static const float RotationMatrix[3][3];    ///<��ת���������ǵ���̨ǹ�ڷ�������û�� 
    //����״̬
    uint8_t shoot_is_permitted=0; ///<�����������ָʾλ��ֻ��ͨ��ShooterSwitchCmd������Ϊ0ʱ������ʹ��Ħ���ֺͲ��������
    uint8_t forced_ctrl_mode = (uint8_t)auto_cloud; // ǿ��ָ���Ŀ���ģʽ
    //PITCH��������
    float g_A=0;  //��������֮ϵ��
    float g_phi=0;    //������������
    //����ϵͳ���
    int RobotHP;    //���ڵ�HP
    //�Ӿ�С����ͨѶ���
    //���CANͨѶ���
};

extern SentryCloud CloudEntity; ///��̨����ʵ����󡣰���������������豸��

// �豸�������߼����
extern CheckDevice_Type UpCloudLeftFric_CheckDevice;
extern CheckDevice_Type UpCloudRightFric_CheckDevice;
extern CheckDevice_Type UpCloudYawMotor_CheckDevice;
extern CheckDevice_Type UpCloudPitchMotor_CheckDevice;
extern CheckDevice_Type UpCloudFeedMotor_CheckDevice;

// ��̨����ģʽ���
//void EnterModeCloudCtrlMech(void);
//void RunModeCloudCtrlMech(void);
//void EnterModeCloudCtrlGyro(void);
//void RunModeCloudCtrlGyro(void);

//extern Mode ModeCloudCtrlMech;  // ��е��λ�û����������ٶȻ�
//extern Mode ModeCloudCtrlGyro;  // ������ λ�û�&�ٶȻ�

/// PID����ص�������PITCH����ǰ�� �߼�
void pidPitchCallBack(pid* self);

#endif // __SENTRY_CLOUD_HPP_
