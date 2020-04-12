/**
 * 
 * @file SentryCloud.cpp
 * @brief    哨兵云台电机控制集合 Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#include "SentryDownCloud.hpp"

// ---------------------------------离线检测用 函数定义---------------------------
// ?? 使用用了大量宏定义，请小心阅读
// 宏定义(1)：定义函数：调用 MotorObj 的 isOffline。将此函数指针传给 CheckDevice_Type 
#define DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(MotorObj)	\
uint8_t func_DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ_##MotorObj(void) \
{		\
	return DownCloudEntity.MotorObj.Is_Offline();	\
}

// 宏定义(2)：给出上面定义的函数的函数名
#define FUNC_NAME(MotorObj) \
	func_DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ_##MotorObj

// (3):使用宏定义(1) 定义函数
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(PitchMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(YawMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(FricLeftMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(FricRightMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(Feed2nd)

//定义各个设备结构体，他们使用了(3)中定义的函数的指针
CheckDevice_Type UpCloudLeftFric_CheckDevice(
                (CheckDeviceID_Enum)         UpCloudLeftFricDevice,  // 设备ID
                                            100,                    // 允许离线时间
                                            FUNC_NAME(FricLeftMotor) ); // 离线检测函数 函数名
CheckDevice_Type UpCloudRightFric_CheckDevice(UpCloudRightFricDevice,100,FUNC_NAME(FricRightMotor));
CheckDevice_Type UpCloudYawMotor_CheckDevice(UpCloudYawMotorDevice,100,FUNC_NAME(YawMotor));
CheckDevice_Type UpCloudPitchMotor_CheckDevice(UpCloudPitchMotorDevice,100,FUNC_NAME(PitchMotor));
CheckDevice_Type UpCloudFeedMotor_CheckDevice(UpCloudFeedMotorDevice,100,FUNC_NAME(Feed2nd));

// ---------------------------------云台 机械角控制&陀螺仪控制 相关---------------------------
//app_Mode ModeCloudCtrlMech(EnterModeCloudCtrlMech,RunModeCloudCtrlMech,nullptr);
//app_Mode ModeCloudCtrlGyro(EnterModeCloudCtrlGyro,RunModeCloudCtrlGyro,nullptr);

void EnterModeCloudCtrlMech(void)
{
    DownCloudEntity.TargetPitch = DownCloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    DownCloudEntity.TargetYaw = DownCloudEntity.RealYaw;
    DownCloudEntity.Mode = absolute_cloud; //视为绝对角控制
}
void RunModeCloudCtrlMech(void)
{
    
}
void EnterModeCloudCtrlGyro(void);
void RunModeCloudCtrlGyro(void);



// 电机型号类
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508_Fric(8192, 1);

// 电机实体定义 >>>>>>>>>>>>>>>>重要<<<<<<<<<<<<<
SentryCloud DownCloudEntity(1, 0x206, 1, 0x205, 1, 0x202, 1, 0x203, 1, 0x204);


/// 云台物理实体类 构造与删除函数
SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)
        // 初始化各项PID参数
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
        // 初始化各电机参数
	  YawMotor(yaw_can_num, yaw_can_id, 4920, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &BaseImuAngleRate[2]),
      PitchMotor(pitch_can_num, pitch_can_id, 0, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{
	Feed2nd.Enable_Block(4000,200,5);                       // 初始化堵转检测
	PitchPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;  // 设定微分来源为陀螺仪
	YawPosition.Custom_Diff = YawMotor.Gyro_RealSpeed;      // 设定微分来源为陀螺仪
    PitchPosition.pid_run_CallBack = pidPitchCallBack;  //位置环PID的用户自定义回调函数。加入重力前馈函数。
    PitchGyroPosition.pid_run_CallBack = pidPitchCallBack;  //位置环PID的用户自定义回调函数。加入重力前馈函数。
};
void SentryCloud::Handle()
{
	if(Mode != save_cloud)
		LazerSwitchCmd(1);  
    else
        LazerSwitchCmd(0);  //安全模式激光灯自动关闭
	
	
	if(Mode != relative_gyro_cloud && Mode != absolute_gyro_cloud)  //不在陀螺仪控制模式中时，陀螺仪角度始终跟随机械角角度（但是要旋转回陀螺仪角度）
	{
		app_imu_data.integral.Pitch = -DownCloudEntity.PitchMotor.RealAngle;//注意负号。
		app_imu_data.integral.Yaw = -DownCloudEntity.YawMotor.RealAngle;//注意负号。
	}
	//↓↓↓陀螺仪角度旋转到枪口方向↓↓↓
	RotatedImuAngle[0] = -app_imu_data.integral.Roll;
    RotatedImuAngle[1] = -app_imu_data.integral.Pitch;
	RotatedImuAngle[2] = -app_imu_data.integral.Yaw;
	//↓↓↓陀螺仪加速度旋转到枪口方向↓↓↓
	RotatedImuAngleRate[0] = -app_imu_data.Angle_Rate[0];
    RotatedImuAngleRate[1] = -app_imu_data.Angle_Rate[1];
	RotatedImuAngleRate[2] = -app_imu_data.Angle_Rate[2];
    //↓↓↓陀螺仪角速度旋转到炮塔方向，即Roll,Pitch水平，Yaw随枪口Yaw↓↓↓
    float Cp = cosf(RealPitch), Sp = sinf(RealPitch);

    BaseImuAngleRate[0] = Cp*RotatedImuAngleRate[0] + Sp*RotatedImuAngleRate[2];
    BaseImuAngleRate[1] = RotatedImuAngleRate[1];
    BaseImuAngleRate[2] = -Sp*RotatedImuAngleRate[0] + Cp*RotatedImuAngleRate[2];
    //更新云台的Yaw,Pitch角度
	RealYaw = YawMotor.RealAngle;   //就是电机的角度
	MechanicYaw = YawMotor.RealPosition*360.f/YawMotor.MotorType->max_mechanical_position;//根据机械角计算出的真实角度
	RealPitch = - PitchMotor.RealAngle;	//注意负号

//CANSend会在主逻辑统一调用
//    manager::CANSend();

	if(feed_is_permitted <1)   //检查射击许可，不许可 则回到安全
	{
		FricLeftMotor.Safe_Set();
		FricRightMotor.Safe_Set();
		Feed2nd.Safe_Set();
	}

    shoot_flag = Feed2nd.feed_mode; //写入射击状态为拨弹电机状态
}
///设定机械角控制角度
void SentryCloud::SetAngleTo(float pitch, float yaw)
{
	Mode = absolute_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
    YawMotor.Angle_Set(TargetYaw);
}
///设定陀螺仪控制角度
void SentryCloud::SetAngleTo_Gyro(float pitch, float yaw)
{
	Mode = absolute_gyro_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Gyro_Angle_Set(-TargetPitch);
    YawMotor.Gyro_Angle_Set(TargetYaw);
}
///安全模式
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

//供弹函数经过包装。需要shoot_is_permitted==1才能运行供弹电机
void SentryCloud::Feed_Free_Fire_Set(int32_t FreeSpeed){
    if(feed_is_permitted)
    Feed2nd.Free_Fire_Set(FreeSpeed);
}
void SentryCloud::Feed_Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig){
    if(feed_is_permitted)
    Feed2nd.Burst_Set(ShootCnt,DiscreDelay,trig);
}
void SentryCloud::Feed_Free_Once_Set(int32_t	DiscreDelay,int16_t trig){
    if(feed_is_permitted)
    Feed2nd.Free_Once_Set(DiscreDelay,trig);
}
void SentryCloud::Feed_Safe_Set(){
    Feed2nd.Safe_Set();
}
///激光灯开关
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
///射击许可开关，摩擦轮开关
void SentryCloud::ShooterSwitchCmd(int NewState )
{
	if(NewState == 0)
	{
		feed_is_permitted=0;    //不允许射击
        FricLeftMotor.Safe_Set();
        FricRightMotor.Safe_Set();
        DownCloudEntity.Feed2nd.Safe_Set();
	}
	else
	{
		feed_is_permitted=1;
        FricLeftMotor.Speed_Set(-Shoot_Speed);
        FricRightMotor.Speed_Set(Shoot_Speed);
	}
}
/**
 * @brief 
 * 
 */
void pidPitchCallBack(pid* self)
{
    self->PIDout+=DownCloudEntity.gravity_feedforward(DownCloudEntity.RealPitch);
}

#undef DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ
#undef FUNC_NAME





































































 

