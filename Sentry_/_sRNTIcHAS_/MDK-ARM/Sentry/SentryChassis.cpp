/**
  * @file  SentryChassis.cpp
  * @brief    哨兵底盘类
  * @details  
  * @author   ThunderDoge
  * @date     2019/11/
  * @version  v1.2.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  *     2019/12/23  v1.2.0  加入功率控制，跟bsp_motor联动 参见宏 __RM2020_SENTRY
  */
#include "SentryChassis.hpp"

//电机类型
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508(8192, 19);
float SpeedMax = 16000;
//系统变量
SentryChassis *SentryChassis::pointer=NULL;
//本体
SentryChassis ChassisEntity(2, 0x201,
                   2, 0x203,
                   1, 0x202,
                   1, 0x201,
                   1, 0x203);
//初始化函数
SentryChassis::SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id,
                             uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
                             uint8_t up_feed_can_num, uint16_t up_feed_can_id,
                             uint8_t down_feed_can_num, uint16_t down_feed_can_id,
                             uint8_t up_fric_can_num, uint16_t up_fric_can_id)
    : pidDriveSpeed(1, 0, 0, 1000, 16000, 100, 300),
      pidDriveLocation(0.035, 0, 0, 1000, SpeedMax, 10, 200),
      FricSpeed(20, 0, 1, 1000, 7000),
      FricLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
      FeedUpSpeed(20, 0, 1, 1000, 7000),
      FeedUpLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
      FeedDownSpeed(20, 0, 1, 1000, 7000),
      FeedDownLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
      pidDriveCurrent(2,0.1, 0, 2500, 10000, 10, 10),
      pidPowerFeedback(0,0,0,1000,10000,10,10),
      DriveWheel(drive_can_num, drive_can_id, &DJI_3508, &pidDriveSpeed, &pidDriveLocation),
      Fric(up_fric_can_num, up_fric_can_id, &DJI_2006, &FricSpeed, &FricLocation),
      FeedUp(up_feed_can_num, up_feed_can_id, &DJI_2006, 7, -1, &FeedUpSpeed, &FeedUpLocation),
      FeedDown(down_feed_can_num, down_feed_can_id, &DJI_2006, 7, -1, &FeedDownSpeed, &FeedDownLocation)
{
    FeedUp.Enable_Block(4000, 200, 5);
    FeedDown.Enable_Block(4000, 200, 5);
    pointer = this; //初始化全局底盘指针
    app_math_Lpf2set(&lpf , 1000.0f, 10.0f);
};

void SentryChassis::Handle()
{
    MotorSpeed = DriveWheel.RealSpeed;
    MotorSoftLocation = DriveWheel.SoftAngle;
	RealPosition = bsp_encoder_Value;
	RealSpeed = bsp_encoder_Speed;
    DrivePower = fabs(bsp_CurrentRead[1] * bsp_VoltageRead[1] / 1000000.0f);
	Accel_Railward = app_imu_data.original.Accel[0];
//    FeedUp.PR_Handle();
//    FeedDown.PR_Handle();
	//撞柱标志
	if( fabs(Accel_Railward) > fabs(imuLeftBounceThreshold) )	//超出阈值
	{
		if( SIGN(Accel_Railward) * SIGN(imuLeftBounceThreshold) == 1)	//同号
			PillarFlag = PILLAR_BOUNCE_LEFT;	//判为左
		else
			PillarFlag = PILLAR_BOUNCE_RIGHT;	//异号判为右
	}
	else
		PillarFlag = PILLAR_NOT_DETECTED;	//未超阈值
		
    manager::CANSend();
}
void SentryChassis::Safe_Set()
{
    DriveWheel.Speed_Set(0);
    Mode = _chassis_save;
}
void SentryChassis::MotorSpeed_Set(float speed_motor_rpm)
{
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Speed_Set(speed_motor_rpm);
    Mode = _chassis_speed;
}
void SentryChassis::MotorSoftLocation_Set(float location_motor_soft)
{
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Angle_Set(location_motor_soft);
    Mode = _chassis_location;
}
void SentryChassis::MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit)
{
    LastMode = Mode;
    pidDriveLocation.PIDMax = speed_motor_rpm_limit;
    DriveWheel.Angle_Set(location_motor_soft);
    Mode = _chassis_location_limit_speed;
}
//#define DEBUG1
//#define DEBUG2
//#define DEBUG3
//#define DEBUG4
#define DEBUG5

#ifdef DEBUG1   
float unfilted_pwr_in;
#endif // DEBUG1    
#ifdef DEBUG3
float P1,P2,P_IdlePower,P_OverPower;
float VeP1,P_Idle;
#endif
#ifdef DEBUG4
#endif
float pwr_exceed,pwr_idle,ft,et,ct;
/**
  * @brief  柴小龙式功率闭环
  * @details  
  *     G(s) = Cur(s)/( 1+Pfd(s)Cur(s) )
  *     Targ    (+) -> Cur(s) -> C(s) -> Motor
  *             ^                 |
  *             |--- Pfd(s) ------+
  * 
  */
float pwr_idle_I=-1;
float SentryChassis::PowerFeedbackSystem(float TargetSpeedInput, float TargetCurInput,float PwrFeedbackInput)
{
	pwr_exceed  = PwrFeedbackInput-LimitPower;
	pwr_idle = -pwr_exceed;		
	(pwr_idle >0)? NULL:pwr_idle =0;//计算空余功率
	(pwr_exceed>0)? NULL: pwr_exceed=0;   //计算超越功率
	if(pwr_idle>1)
	{
		pidPowerFeedback.Iout-=pwr_idle_I*pwr_idle;
		if(pidPowerFeedback.Iout<0)	pidPowerFeedback.Iout=0;
	}
	ft = SIGN(TargetSpeedInput) * pidPowerFeedback.pid_run(pwr_exceed);
	et = TargetCurInput+ft;
	ct = pidDriveCurrent.pid_run(et);
    return ct;
}

/**
  * @brief  底盘功率限制
  * @details  
  * @param[in]  
  * @retval  
  */
void SentryChassis::CanSendHandle()
{
	if(Mode != _chassis_save)
    if (LimitPower > 0)
    {   //只有大于0才会启动
#ifdef DEBUG1
        if(HAL_GetTick() - PwrUpdateTime > 0){
        unfilted_pwr_in = (((float)DriveWheel.TargetCurrent) /819.2f) //目标电流
                            *(((float)bsp_VoltageRead[1]) / 1000.0f);   //检测电压
        TargetPowerInput = app_math_Lpf2apply(&lpf , unfilted_pwr_in);  //低通滤波防抖
        }
        if (fabs(TargetPowerInput) > LimitPower)    //功率限幅
        {
            TargetPowerInput = LimitPower* SIGN(TargetPowerInput);
        }
//        if(SIGN(TargetPowerInput) * SIGN(PowerOutput) == -1)    //功率防反向
//        {
//            PowerOutput = 0;
//        }
        if(HAL_GetTick() - PwrUpdateTime > 0){      //防止积分频率不稳定
            PowerOutput += pidDriveCurrent.pid_run(TargetPowerInput - DrivePower);
            PwrUpdateTime = HAL_GetTick();
        }
        DriveWheel.TargetCurrent = PowerOutput;
#endif
#ifdef DEBUG2
        TargetPowerInput = (((float)DriveWheel.TargetCurrent) /819.2f) //目标电流
                            *(((float)bsp_VoltageRead[1]) / 1000.0f);   //检测电压
		if (fabs(TargetPowerInput) > LimitPower)    //功率限幅
		{
			DriveWheel.TargetCurrent *= (LimitPower/TargetPowerInput);
		}
#endif
#ifdef DEBUG3
    VeP1 = (DriveWheel.TargetSpeed - DriveWheel.RealSpeed)*P1;
    P_Idle = LimitPower - DrivePower;
    if(P_Idle>0){
        P2 = P_Idle * P_IdlePower;
        PowerOutput += VeP1 * P2;
    }
    else{
        PowerOutput += P_Idle*P_OverPower;
    }
    DriveWheel.TargetCurrent = PowerOutput;
#endif // DEBUG3
#ifdef DEBUG4
	TargetPowerInput = DriveWheel.TargetCurrent - DriveWheel.RealCurrent;
	TargetPowerInput = app_math_Lpf2apply(&lpf,TargetPowerInput);
	PowerOutput = pidDriveCurrent.pid_run(TargetPowerInput);
	if(fabs(DrivePower) > LimitPower)
	{
		PowerOutput *= LimitPower/fabs(DrivePower); 
	}
    DriveWheel.TargetCurrent = PowerOutput;
#endif	//DEBUG4
#ifdef DEBUG5
	PowerOutput = PowerFeedbackSystem (DriveWheel.TargetSpeed,DriveWheel.TargetCurrent,DrivePower);
#endif	//DEBUG5
    DriveWheel.InsertCurrentBy(PowerOutput);
    }
	else
	{
	DriveWheel.InsertCurrent();
	}
}
#undef DEBUG1
#undef DEBUG2

