/**
 * @brief    哨兵云台电机控制集合 Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#include "SentryCloud.hpp"


Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508_Fric(8192, 1);

SentryCloud Self(1, 0x206, 1, 0x205, 1, 0x202, 1, 0x203, 1, 0x201);

SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)

    : PitchSpeed(-4, 0, 0, 2000, 30000, 10, 10, 500), 
	  PitchPosition(-16, -0.5, 0, 5000, 10000, 10, 10, 150),
      PitchGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      PitchGyroSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
      YawSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(10, 0, 0, 2000, 10000, 10, 10, 3000),
      YawGyroSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
      FeedSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
      FeedPositon(0, 0, 0, 2000, 10000, 10, 10, 3000),

      YawMotor(yaw_can_num, yaw_can_id, 4920, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &RotatedImuAngle[2]),
      PitchMotor(pitch_can_num, pitch_can_id, 0, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{};
void SentryCloud::Handle()
{
	RotatedImuAngle[0] = app_imu_data.integral.Pitch;
    RotatedImuAngle[1] = app_imu_data.integral.Roll;
	RotatedImuAngle[2] = -app_imu_data.integral.Yaw;
	
	RotatedImuAngleRate[0] = app_imu_data.Angle_Rate[1];
    RotatedImuAngleRate[1] = app_imu_data.Angle_Rate[0];
	RotatedImuAngleRate[2] = -app_imu_data.Angle_Rate[2];
	RealYaw = YawMotor.RealAngle;
	RealPitch = - PitchMotor.RealAngle;	//注意负号
    manager::CANSend();
}

void SentryCloud::SetAngleTo(float pitch, float yaw)
{
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
    YawMotor.Angle_Set(TargetYaw);
}

void SentryCloud::Safe_Set()
{
    YawMotor.Safe_Set();
    PitchMotor.Safe_Set();
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
    manager::CANSend();
}
