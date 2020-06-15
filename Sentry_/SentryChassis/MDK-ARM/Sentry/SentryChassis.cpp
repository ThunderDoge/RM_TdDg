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
SentryChassis ChassisEntity(1, 0x201);
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
    : pidDriveSpeed(1, 0, 0, 0, 5000, 100, 300),
      pidDriveLocation(0.035, 0, 0, 1000, SpeedMax, 10, 200),
      pidPwrFdbkDriveSpeed(0,0,0,0,1000,10,10),
      pidPwrFdbkDriveCurrent(2,0.1, 0, 0, 0, 10, 10),
      pidPowerFeedback(0,0,0,0,1000,10,10),
      DriveWheel(drive_can_num, drive_can_id, &DJI_3508, &pidDriveSpeed, &pidDriveLocation)
{
    pointer = this; //初始化全局底盘指针
};

void SentryChassis::Handle()
{
    MotorSpeed = DriveWheel.RealSpeed;
    MotorSoftLocation = DriveWheel.SoftAngle;
	RealPosition = bsp_encoder_Value;
	RealSpeed = bsp_encoder_Speed;

    // #if defined(__APP_CHECK_H) && defined(_JUDGEMENT_H_)
    //     if(app_check_IsEnabled(id_Judge) && !app_check_IsOffline(id_Judge))
    //     {
    //         DrivePower = power_heat_data.chassis_power;
    //     }
    //     else
        // {
            DrivePower = fabs(bsp_CurrentRead[1] * bsp_VoltageRead[1] / 1000000.0f);
        // }
    // #else
        DrivePower = fabs(bsp_CurrentRead[1] * bsp_VoltageRead[1] / 1000000.0f);
    // #endif // defined(__APP_CHECK_H) && _defined(JUDGEMENT_H_)

    ChassisModeCtrl();

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
    ChassisMode = _chassis_safe;
}
void SentryChassis::MotorSpeed_Set(float speed_motor_rpm)
{
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Speed_Set(speed_motor_rpm);
    ChassisMode = _chassis_speed;
}
void SentryChassis::MotorSoftLocation_Set(float location_motor_soft)
{
    pidDriveLocation.PIDMax = SpeedMax;
    DriveWheel.Angle_Set(location_motor_soft);
    ChassisMode = _chassis_location;
}
void SentryChassis::MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit)
{
    LastMode = ChassisMode;
    pidDriveLocation.PIDMax = speed_motor_rpm_limit;
    DriveWheel.Angle_Set(location_motor_soft);
    ChassisMode = _chassis_location_limit_speed;
}
void SentryChassis::Safe_Set_NoMode()
{
    DriveWheel.Speed_Set(0);
}

float pwr_exceed,pwr_idle,ft,et,ct;
/**
  * @brief  柴小龙式功率闭环
  * @details  
  *     G(s) = Cur(s)/( 1+Pfd(s)Cur(s) )
  *     Targ    (+) -> Cur(s) -> C(s) -> Motor
  *             ^                 |
  *             |--- Pfd(s) ------+
  * 
  * @param 
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
	ct = pidPwrFdbkDriveCurrent.pid_run(et);
    return ct;
}




uint8_t max_to_cur;
float SpdpidMax,SpdOut,CurOut,CurErr;
/**
 * @brief 2019柴小龙英雄功率控制复刻
 * 
 */
float SentryChassis::PowerCtrMot_CascadePidRegular(float TargetSpeedInput)
{
    if(max_to_cur){
        SpdpidMax = pidPwrFdbkDriveCurrent.PIDMax = pidPowerFeedback.pid_inc_run(LimitPower - DrivePower);
    }else{
        SpdpidMax = DriveWheel.PID_In->PIDMax = pidPowerFeedback.pid_inc_run(LimitPower - DrivePower);
    }
    SpdOut = DriveWheel.TargetCurrent = DriveWheel.PID_In->pid_run(TargetSpeedInput - DriveWheel.RealSpeed);
    return CurOut = pidPwrFdbkDriveCurrent.pid_run((CurErr = (DriveWheel.TargetCurrent/819.2f)-(bsp_CurrentRead[1]/1000.0f)));
}

/**
 * @brief 底盘模式管理
 * 
 */
void SentryChassis::ChassisModeCtrl()
{
    switch (ChassisMode)
    {
    case _chassis_speed:
    case _chassis_location:
    case _chassis_location_limit_speed:
    case _chassis_safe:
        break;

    default:
        ChassisMode = _chassis_default;

    case _chassis_default:
        Safe_Set_NoMode();

        break;
    }
    LastMode = ChassisMode;

    if(EnablePowerCtrl && !LastEnablePowerCtrl)
    {
        DriveWheel.PID_In = &pidPwrFdbkDriveSpeed;
    }
    if(!EnablePowerCtrl && LastEnablePowerCtrl)
    {
        DriveWheel.PID_In = &pidDriveSpeed;
    }

    LastEnablePowerCtrl = EnablePowerCtrl;
}


/**
 * @brief 底盘功率限制
 * 
 */
void SentryChassis::CanSendHandle()
{
	if(ChassisMode != _chassis_safe)
    if ( EnablePowerCtrl && (LimitPower > 0) )
    {   //只有大于0才会启动
        switch (use_system)
        {
        case 1:
            PwrCtrlCurrentOutput = PowerFeedbackSystem (DriveWheel.TargetSpeed,DriveWheel.TargetCurrent,DrivePower);
            DriveWheel.InsertCurrentBy(PwrCtrlCurrentOutput);
            break;
        case 2:
            PwrCtrlCurrentOutput = PowerCtrMot_CascadePidRegular(DriveWheel.TargetSpeed);
            DriveWheel.InsertCurrentBy(PwrCtrlCurrentOutput);
            break;
        default:
            DriveWheel.InsertCurrent();
            EnablePowerCtrl = 0;
            use_system = 0;
            break;
        }
    }
	else
	{
        DriveWheel.InsertCurrent();
	}
}

