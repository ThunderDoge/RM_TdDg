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
    //------------------------系统变量
    static SentryChassis* pointer;
    //-------------------------PID变量
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
    //-------------------------电机变量
    softmotor DriveWheel;
    motor Fric;
    AmmoFeed FeedUp;
    AmmoFeed FeedDown;
    //-------------------------状态参数
    int Mode;
    int LastMode;
    float MotorSpeed;        //电机反馈速度，单位rpm
    float MotorSoftLocation; //电机软路程，单位是角度
    float RealSpeed;    //真正的速度，单位mm/s
    float RealPosition; //真正的距离，单位mm
	//-------------------------功率计算用参数

    float DrivePower;      //驱动轮功率，单位W
    float LimitPower = -1; //底盘限制功率
    //-------------------------操作函数
    void Handle();                                         //更新函数
    void Safe_Set();                                       //安全模式
    void MotorSpeed_Set(float speed_motor_rpm);            //设定电机速度
    void MotorSoftLocation_Set(float location_motor_soft); //设定电机软路程
    void MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit);
    // void Speed_Set(float speedBy_mm_s); //设定真实速度
    // void Location_Set(float locationBy_mm); //设定真实位置。
private:
    //功率计算用变量
    float TargetPowerInput = 0;
    float PowerOutput = 0;
    uint32_t PwrUpdateTime = 0;
    LPF2 lpf;


    void CanSendHandle(); //托管到CANSend的操作函数
    float SentryChassis::PowerFeedbackSystem(float TargetCurInput,float PwrFeedbackInput);//柴小龙式功率闭环

    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
};
extern SentryChassis Self;

#endif // __SENTRY_CHASSIS_HPP_
