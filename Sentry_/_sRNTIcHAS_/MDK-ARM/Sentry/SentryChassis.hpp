#ifndef __SENTRY_CHASSIS_HPP_
#define __SENTRY_CHASSIS_HPP_

#include "app_AmmoFeed.hpp"
#include "app_imu.h"
#include "bsp_motor.hpp"
#include "bsp_current.h"

extern float SpeedMax;

enum PillarActionStateName:int8_t
{
    PILLAR_NOT_DETECTED,
    PILLAR_DETECTED_NO_ACT,
    PILLAR_APROACHING,
    PILLAR_LEAVING,
    PILLAR_BOUNCE,
};
enum SentryChassisMode
{
    SENTRY_SAFE=1,
    SENTRY_SLAVE,
    SENTRY_AUTO,
};


class SentryChassis
{
public:

    SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id,
                  uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
                  uint8_t up_feed_can_num, uint16_t up_feed_can_id,
                  uint8_t down_feed_can_num, uint16_t down_feed_can_id,
                  uint8_t up_fric_can_num, uint16_t up_fric_can_id);
    //-------------------------PID����
    pid pidDriveSpeed;
    pid pidDriveLocation;
	pid FricSpeed;
	pid FricLocation;
	pid FeedUpSpeed;
	pid FeedUpLocation;
	pid FeedDownSpeed;
	pid FeedDownLocation;
    //-------------------------�������
    softmotor DriveWheel;
	motor Fric;
	AmmoFeed FeedUp;
	AmmoFeed FeedDown;
    //-------------------------����
    int Mode;
    float MotorSpeed;   //��������ٶȣ���λrpm
    float MotorSoftLocation;    //�����·�̣���λ�ǽǶ�
    float RealSpeed;    //�������ٶȣ���λmm/s
    float RealPosition; //�����ľ��룬��λmm
	float DrivePower;	//�����ֹ��ʣ���λW
    //-------------------------��������
    void Handle();      //���º���
    void Safe_Set();    //��ȫģʽ
    void MotorSpeed_Set(float speed_motor_rpm); //�趨����ٶ�
    void MotorSoftLocation_Set(float location_motor_soft);  //�趨�����·��
    // void Speed_Set(float speedBy_mm_s); //�趨��ʵ�ٶ�
    // void Location_Set(float locationBy_mm); //�趨��ʵλ�á�
private:
    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
    // PillarActionStateName ActionStateWithPillar;
};


extern SentryChassis Self;

#endif // __SENTRY_CHASSIS_HPP_
