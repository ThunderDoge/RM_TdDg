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
    //-------------------------PID变量
    pid pidDriveSpeed;
    pid pidDriveLocation;
	pid FricSpeed;
	pid FricLocation;
	pid FeedUpSpeed;
	pid FeedUpLocation;
	pid FeedDownSpeed;
	pid FeedDownLocation;
    //-------------------------电机变量
    softmotor DriveWheel;
	motor Fric;
	AmmoFeed FeedUp;
	AmmoFeed FeedDown;
    //-------------------------参数
    int Mode;
    float MotorSpeed;   //电机反馈速度，单位rpm
    float MotorSoftLocation;    //电机软路程，单位是角度
    float RealSpeed;    //真正的速度，单位mm/s
    float RealPosition; //真正的距离，单位mm
	float DrivePower;	//驱动轮功率，单位W
    //-------------------------操作函数
    void Handle();      //更新函数
    void Safe_Set();    //安全模式
    void MotorSpeed_Set(float speed_motor_rpm); //设定电机速度
    void MotorSoftLocation_Set(float location_motor_soft);  //设定电机软路程
    // void Speed_Set(float speedBy_mm_s); //设定真实速度
    // void Location_Set(float locationBy_mm); //设定真实位置。
private:
    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
    // PillarActionStateName ActionStateWithPillar;
};


extern SentryChassis Self;

#endif // __SENTRY_CHASSIS_HPP_
