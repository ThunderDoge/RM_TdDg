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

SentryCloud Self(1, 0x206, 1, 0x205, 1, 0x202, 1, 0x203, 1, 0x204);

SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)

    : PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
	  PitchPosition(-15, -1, 0, 1800, 10000, 10, 10, 120),//(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
      PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),
      YawSpeed(20, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(10, 1,-0.5, 200, 10000, 10, 2, 100),//10, 0, 0, 2000, 10000, 10, 10, 3000)
      YawGyroSpeed(15, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FeedSpeed(20, 0, 1, 1000, 7000),
      FeedPositon(0.5, 0.01, 0, 1000, 20000, 0, 200),
	  
	  YawMotor(yaw_can_num, yaw_can_id, 4920, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &BaseImuAngleRate[2]),
      PitchMotor(pitch_can_num, pitch_can_id, 0, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{
	Feed2nd.Enable_Block(4000,200,5);
	PitchPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;
	YawPosition.Custom_Diff = YawMotor.Gyro_RealSpeed;
};
void SentryCloud::Handle()
{
	if(Mode != save_cloud){
		LazerSwitchCmd(1);
	}
	
	if(Mode != relative_gyro_cloud && Mode != absolute_gyro_cloud)
	{
		app_imu_data.integral.Pitch = -Self.PitchMotor.RealAngle; 
		app_imu_data.integral.Yaw = -Self.YawMotor.RealAngle;//注意负号。
	}
	
	RotatedImuAngle[0] = -app_imu_data.integral.Roll;
    RotatedImuAngle[1] = -app_imu_data.integral.Pitch;
	RotatedImuAngle[2] = -app_imu_data.integral.Yaw;
	
	RotatedImuAngleRate[0] = -app_imu_data.Angle_Rate[0];
    RotatedImuAngleRate[1] = -app_imu_data.Angle_Rate[1];
	RotatedImuAngleRate[2] = -app_imu_data.Angle_Rate[2];

    float Cp = cosf(RealPitch), Sp = sinf(RealPitch);

    BaseImuAngleRate[0] = Cp*RotatedImuAngleRate[0] + Sp*RotatedImuAngleRate[2];
    BaseImuAngleRate[1] = RotatedImuAngleRate[1];
    BaseImuAngleRate[2] = -Sp*RotatedImuAngleRate[0] + Cp*RotatedImuAngleRate[2];

	RealYaw = YawMotor.RealAngle;
	MechanicYaw = YawMotor.RealPosition*360.f/YawMotor.MotorType->max_mechanical_position;//根据机械角计算出的真实角度
	RealPitch = - PitchMotor.RealAngle;	//注意负号
//    manager::CANSend();
	if(shoot_flag <1)
	{
		FricLeftMotor.Safe_Set();
		FricRightMotor.Safe_Set();
		Feed2nd.Safe_Set();
	}
}

void SentryCloud::SetAngleTo(float pitch, float yaw)
{
	Mode = absolute_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
    YawMotor.Angle_Set(TargetYaw);
}
void SentryCloud::SetAngleTo_Gyro(float pitch, float yaw)
{
	Mode = absolute_gyro_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Gyro_Angle_Set(-TargetPitch);
    YawMotor.Gyro_Angle_Set(TargetYaw);
}

void SentryCloud::Safe_Set()
{
	Mode = save_cloud;
    YawMotor.Safe_Set();
//    PitchMotor.Safe_Set();
	PitchMotor.Speed_Set(0);
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
    manager::CANSend();
	LazerSwitchCmd(0);
}

void SentryCloud::LazerSwitchCmd( int NewState )
{
	if(NewState == 0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	}
}







































































 

