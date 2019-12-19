#include "SentryChassis.hpp"

//电机类型
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508(8192, 19);
float SpeedMax = 16000;
//本体
SentryChassis Self(2, 0x201,
                   2, 0x203,
                   1, 0x202,
                   1, 0x201,
                   1, 0x203);
//初始化函数
SentryChassis::SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id,
                             uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
                             uint8_t up_feed_can_num, uint16_t up_feed_can_id,
                             uint8_t down_feed_can_num,uint16_t down_feed_can_id, 
							 uint8_t up_fric_can_num,uint16_t up_fric_can_id)
    : pidDriveSpeed(1, 0, 0, 1000, 16000, 100, 300),
      pidDriveLocation(0.035, 0, 0, 1000, SpeedMax, 10, 200),
	  FricSpeed(20,0,1,1000,7000),
	  FricLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
	  FeedUpSpeed(20,0,1,1000,7000),
	  FeedUpLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
	  FeedDownSpeed(20,0,1,1000,7000),
	  FeedDownLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
      DriveWheel(drive_can_num, drive_can_id, &DJI_3508, &pidDriveSpeed, &pidDriveLocation),
	  Fric(up_fric_can_num,up_fric_can_id,&DJI_2006,&FricSpeed,&FricLocation),
	  FeedUp(up_feed_can_num,up_feed_can_id,&DJI_2006,7,-1,&FeedUpSpeed,&FeedUpLocation),
	  FeedDown(down_feed_can_num,down_feed_can_id,&DJI_2006,7,-1,&FeedDownSpeed,&FeedDownLocation)
	  {
		FeedUp.Enable_Block(4000,200,5);
		FeedDown.Enable_Block(4000,200,5);
	  };

void SentryChassis::Handle()
{
    MotorSpeed = DriveWheel.RealSpeed;
    MotorSoftLocation = DriveWheel.SoftAngle;
    manager::CANSend();
	DrivePower = bsp_CurrentRead[1] * bsp_VoltageRead[1] / 1000000.0f;
	FeedUp.PR_Handle();
	FeedDown.PR_Handle();
}
void SentryChassis::Safe_Set()
{
    DriveWheel.Speed_Set(0);
    Mode = _chassis_save;
}
void SentryChassis::MotorSpeed_Set(float speed_motor_rpm){
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Speed_Set(speed_motor_rpm);
    Mode = _chassis_speed;
}
void SentryChassis::MotorSoftLocation_Set(float location_motor_soft){
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Angle_Set(location_motor_soft);
    Mode = _chassis_location;
}
void SentryChassis::MotorSoftLocation_LimitSpeed_Set(float location_motor_soft,float speed_motor_rpm_limit)
{
    LastMode = Mode;
    pidDriveLocation.PIDMax = speed_motor_rpm_limit;
    DriveWheel.Angle_Set(location_motor_soft);
    Mode = _chassis_location_limit_speed;
}

