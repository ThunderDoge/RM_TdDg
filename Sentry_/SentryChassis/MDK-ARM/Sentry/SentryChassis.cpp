/**
 * @file        SentryChassis.cpp
 * @brief       哨兵底盘物理实体
 * @details     
 *  哨兵底盘主要功能硬件有：
 *  - 底盘主动轮电机，板上有功率检测芯片检测它的实时电流与功率
 *  - 与底盘固定的陀螺仪
 *  - 沿轨道左右各一：GY-53/VL531LX廉价激光测距模块。测距范围2000mm，精度在10mm
 *  - 沿轨道左右各一：弹簧。允许以不超过X.X mm/s的速度撞击轨道，并保护底盘主体不受撞击
 *      - 上下云台归云台自己控制，底盘不控制。
 *      - 底盘
 *  需要实现以下功能：
 *  - 根据现有传感器尽可能精确估计轨上位置
 *  - 底盘主动轮控制。含功率控制
 * 
 * @author   ThunderDoge
 * @date      2020-4-11
 * @version   v1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 * 注：在2019-12月之前的哨兵底盘具有不同的结构：
 * 弹药舱装在底盘，底盘有两个拨弹轮分别向上下拨弹；另有一摩擦轮辅助向上云台供弹管 推弹丸。
 * 在2019-1月之前被废弃。代码上可能有残留部分，但大部分已经注释。
 *  verison|    date|       author|         change|
 *  1.2         2019-12-13  ThunderDoge     添加了柴小龙式功率控制
 *  2.0         2020-4-11   ThunderDoge     添加了更详细的注释;添加了激光测距模块的支持
 */
#include "SentryChassis.hpp"



float RAIL_LEFT_END_MM = 2250;
float RAIL_RIGHT_END_MM = 0;


float RATIO_ENCODE_PER_MM        =(73.4f);
float RATIO_ENCODE_SPD_PER_MM_S 	=(10.0f);

float RATIO_MOTOR_MM_PER_ANG		=(0.610f);
float RATIO_MOTOR_MM_S_PER_SPD	=(0.02f);

float LAZER_LEFT_RANGE = 2000;
float LAZER_RIGHT_RANGE = 2000;
float LAZER_TOUCH_LEFT = 300;
float LAZER_TOUCH_RIGHT = 300;





//电机类型
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508(8192, 19);
float SpeedMax = 16000;
//系统变量
SentryChassis *SentryChassis::pointer=NULL;
//本体
SentryChassis ChassisEntity(2, 0x201);
//                   2, 0x203,
//                   1, 0x202,
//                   1, 0x201,
//                   1, 0x203);







//初始化函数
SentryChassis::SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id)
//                             uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
//                             uint8_t up_feed_can_num, uint16_t up_feed_can_id,
//                             uint8_t down_feed_can_num, uint16_t down_feed_can_id,
//                             uint8_t up_fric_can_num, uint16_t up_fric_can_id)
    : pidDriveSpeed(1, 0, 0, 1000, 16000, 100, 300),
      pidDriveLocation(0.035, 0, 0, 1000, SpeedMax, 10, 200),
//      FricSpeed(20, 0, 1, 1000, 7000),
//      FricLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
//      FeedUpSpeed(20, 0, 1, 1000, 7000),
//      FeedUpLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
//      FeedDownSpeed(20, 0, 1, 1000, 7000),
//      FeedDownLocation(0.5, 0.01, 0, 1000, 20000, 0, 200),
      pidDriveCurrent(2,0.1, 0, 2500, 10000, 10, 10),
      pidPowerFeedback(0,0,0,1000,10000,10,10),
      DriveWheel(drive_can_num, drive_can_id, &DJI_3508, &pidDriveSpeed, &pidDriveLocation)
//      ,Fric(up_fric_can_num, up_fric_can_id, &DJI_2006, &FricSpeed, &FricLocation),
//      FeedUp(up_feed_can_num, up_feed_can_id, &DJI_2006, 7, -1, &FeedUpSpeed, &FeedUpLocation),
//      FeedDown(down_feed_can_num, down_feed_can_id, &DJI_2006, 7, -1, &FeedDownSpeed, &FeedDownLocation)
{
//    FeedUp.Enable_Block(4000, 200, 5);
//    FeedDown.Enable_Block(4000, 200, 5);
    pointer = this; //初始化全局底盘指针
    app_math_LPF2pSetCutoffFreq(&lpf , 1000.0f, 10.0f);
};

void SentryChassis::Handle()
{
    MotorSpeed = DriveWheel.RealSpeed;
    MotorSoftLocation = DriveWheel.SoftAngle;
	RealPosition = bsp_encoder_Value;
	RealSpeed = bsp_encoder_Speed;
	
    DrivePower = fabs(bsp_CurrentRead[1] * bsp_VoltageRead[1] / 1000000.0f);

    if(Accel_Railward_UseKalman)
    {
        Accel_Railward = app_imu_data.kalman.Accel[0]; // Oringin值过Kalman滤波
    }
    else
    {
        Accel_Railward = app_imu_data.original.Accel[0];    // 此处取Oringin值
    }

    LocationSpeedDataMultiplexer();     // 获取路程和速度。根据设备离线情况.

	//撞柱标志
    PillarFlag = PILLAR_NOT_DETECTED;
    
    // 陀螺仪超过阈值指示撞柱
    if(!app_check_IsOffline(id_ChassisImu)) // 先检查数据是不是有效的
    {
        if(Accel_Railward < -imuAccelHitPillarThreshold[0])     // Accel_Railward是 面对敌方时的左侧为正方向，而imuAccelHitPillarThreshold只有绝对值
        {
            PillarFlag = PILLAR_HIT_LEFT;
            DriveWheel.ForceSetSoftAngle(RAIL_RIGHT_END_MM);
        }
        else if(Accel_Railward > imuAccelHitPillarThreshold[1])
        {
            PillarFlag = PILLAR_HIT_RIGHT;
            DriveWheel.ForceSetSoftAngle(RAIL_RIGHT_END_MM);
        }
    }
    // 激光测距结果小于阈值->指示 接近柱 或 撞柱. 这是左侧的
    if(!app_check_IsOffline(id_ChassisLazerRangingLeft))
    {
        if(LazerRanging[0]<LAZER_LEFT_RANGE)    // LAZER_LEFT_RANGE 超过此值认为是无效的
        {
            if(LazerRanging[0]<LAZER_TOUCH_LEFT)    // LAZER_TOUCH_LEFT 是测试得到的 底盘贴着柱子时的距离值
            {
                PillarFlag = PILLAR_HIT_LEFT;
            }
            PillarFlag = PILLAR_LEFT;
        }
    }
    // 激光测距结果小于阈值 指示 接近柱 或 撞柱. 右侧同理.
    if(!app_check_IsOffline(id_ChassisLazerRangingRight))
    {
        if(LazerRanging[0]<LAZER_RIGHT_RANGE)   
        {
            if(LazerRanging[0]<LAZER_TOUCH_RIGHT)
            {
                PillarFlag = PILLAR_HIT_RIGHT;
            }
            PillarFlag = PILLAR_RIGHT;
        }
    }


	
	manager::CANSend();
}


void SentryChassis::LocationSpeedDataFusion()
{
	
}

void SentryChassis::LocationSpeedDataMultiplexer()
{
    if( !app_check_IsOffline(id_ChassisRailEncoder) )
    {
        RealPosition = bsp_encoder_Value / (RATIO_ENCODE_PER_MM);
        RealSpeed = bsp_encoder_Speed / (RATIO_ENCODE_SPD_PER_MM_S);
    }
    else if(!app_check_IsOffline(id_ChassisDriveMotor))
    {
        RealPosition = DriveWheel.RealAngle * (RATIO_MOTOR_MM_PER_ANG);
        RealSpeed = DriveWheel.RealSpeed * (RATIO_MOTOR_MM_S_PER_SPD);
    }
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

