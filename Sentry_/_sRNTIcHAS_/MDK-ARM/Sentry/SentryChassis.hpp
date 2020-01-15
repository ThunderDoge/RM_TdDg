#ifndef __SENTRY_CHASSIS_HPP_
#define __SENTRY_CHASSIS_HPP_

#include "app_AmmoFeed.hpp"
#include "app_imu.h"
#include "bsp_motor.hpp"
#include "bsp_current.h"
#include "bsp_adc_deal.h"
#include "bsp_encoder.hpp"
#include "app_math.h"
#include <cmath>
#include <cstring>

extern float SpeedMax;

#ifndef __SENTRY_COMMU_HPP_

#ifndef __CHASSIS_MODE_DEF
#define __CHASSIS_MODE_DEF
enum _chassis_mode : uint8_t
{
    _chassis_speed = 1,
    _chassis_location = 2,
    _chassis_location_limit_speed = 3,
    _chassis_save = 0,
};
#endif //__CHASSIS_MODE_DEF
#endif // !__SENTRY_COMMU_HPP_

enum PillarFlage : int8_t
{
    PILLAR_NOT_DETECTED = 0,
    PILLAR_LEFT = 0X01,
    PILLAR_RIGHT = 0X02,
};
// enum SentryChassisMode
// {
//     SENTRY_SAFE=1,
//     SENTRY_SLAVE,
//     SENTRY_AUTO,
// };

class SentryChassis
{
    friend class manager;

public:
    SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id,
                  uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
                  uint8_t up_feed_can_num, uint16_t up_feed_can_id,
                  uint8_t down_feed_can_num, uint16_t down_feed_can_id,
                  uint8_t up_fric_can_num, uint16_t up_fric_can_id);
    //------------------------ϵͳ����
    static SentryChassis* pointer;
    //-------------------------PID����
    pid pidDriveSpeed;
    pid pidDriveLocation;
    pid FricSpeed;
    pid FricLocation;
    pid FeedUpSpeed;
    pid FeedUpLocation;
    pid FeedDownSpeed;
    pid FeedDownLocation;
    pid pidDriveCurrent;
    pid pidPowerFeedback;
    //-------------------------�������
    softmotor DriveWheel;
    motor Fric;
    AmmoFeed FeedUp;
    AmmoFeed FeedDown;
    //-------------------------״̬����
    int Mode;
    int LastMode;
    float MotorSpeed;        //��������ٶȣ���λrpm
    float MotorSoftLocation; //�����·�̣���λ�ǽǶ�
    float RealSpeed;    //�������ٶȣ���λmm/s
    float RealPosition; //�����ľ��룬��λmm
	//-------------------------���ʼ����ò���

    float DrivePower;      //�����ֹ��ʣ���λW
    float LimitPower = -1; //�������ƹ���
    //-------------------------��������
    void Handle();                                         //���º���
    void Safe_Set();                                       //��ȫģʽ
    void MotorSpeed_Set(float speed_motor_rpm);            //�趨����ٶ�
    void MotorSoftLocation_Set(float location_motor_soft); //�趨�����·��
    void MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit);
    // void Speed_Set(float speedBy_mm_s); //�趨��ʵ�ٶ�
    // void Location_Set(float locationBy_mm); //�趨��ʵλ�á�
private:
    //���ʼ����ñ���
    float TargetPowerInput = 0;
    float PowerOutput = 0;
    uint32_t PwrUpdateTime = 0;
    LPF2 lpf;


    void CanSendHandle(); //�йܵ�CANSend�Ĳ�������
    float SentryChassis::PowerFeedbackSystem(float TargetCurInput,float PwrFeedbackInput);//��С��ʽ���ʱջ�

    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
};
extern SentryChassis Self;

#endif // __SENTRY_CHASSIS_HPP_
