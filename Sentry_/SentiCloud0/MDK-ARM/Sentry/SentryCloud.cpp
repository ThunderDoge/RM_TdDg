/**
 * @file SentryCloud.cpp
 * @brief    哨兵上云台
 * @details     Encoding - UTF8
 * 包含：
 * 云台电机控制：左/右俯仰轴、航向轴、左/右摩擦轮、拨弹
 * 云台俯仰轴电机离线检测。如果检测到离线会改为使用未离线的电机进行PID及控制。
 * 航向角 机械角/陀螺仪 灵活切换。因为在某些角度的航向角机械角控制会抖动。
 * @author   ThunderDoge
 * @date     2020-4-15
 * @version  v2.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 * 
 * v2.0 2020-4-15   发布
 */
#include "SentryCloud.hpp"

// ---------------------------------云台 机械角控制&陀螺仪控制 相关---------------------------
app_Mode ModeCloudCtrlMech(EnterModeCloudCtrlMech,RunModeCloudCtrlMech,NULL);
app_Mode ModeCloudCtrlGyro(EnterModeCloudCtrlGyro,RunModeCloudCtrlGyro,NULL);

// ---------------------------------云台 双PITCH相关
//app_Mode ModeDualPitch(EnterModeDualPitch,RunModeDualPitch,NULL);
//app_Mode ModeSinglePitch(EnterModeSinglePitch,RunModeSinglePitch,NULL);

/**
 * @brief 进入机械角
 * 避免突变
 */
void EnterModeCloudCtrlMech(void)
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //视为绝对角控制
}
/**
 * @brief 运行机械角时，数据持续维护 逻辑。
 * 陀螺仪跟随机械角数据
 */
void RunModeCloudCtrlMech(void)
{
    // 平时依赖陀螺仪积分数据。但是有机械角的时候强制陀螺仪跟随机械角数据
    app_imu_data.integral.Pitch = -CloudEntity.PitchMotor.RealAngle;//注意负号。
    app_imu_data.integral.Yaw = -CloudEntity.YawMotor.RealAngle;//注意负号。
}
/**
 * @brief 进入云台陀螺仪控制模式
 * 
 */
void EnterModeCloudCtrlGyro(void)
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //视为绝对角控制

}
/**
 * @brief 运行云台陀螺仪控制模式
 * 
 */
void RunModeCloudCtrlGyro(void)
{

}


                                                                                                              
                                                                                                              
                                                                                                              
                                                                                                              
//    .@@@@@@@@@@]`                              =@@@^   /@@@@@@@@@O]`   =@@@^                      .@@@O        
//    .@@@@@@@@@@@@@`                            =@@@^   @@@@@@@@@@@@@^  =@@@^  =@@@.               .@@@@        
//    .@@@@.    =@@@^ =@@@^   =@@@^  @@@@@@@@@\  =@@@^   @@@@^    ,@@@@.]]]]]`.@@@@@@@@  ./@@@@@@@@^.@@@@@@@@@@\.
//    .@@@@.    =@@@^ =@@@^   =@@@^  @@@@O/@@@@\ =@@@^   @@@@^    .@@@@.@@@@@^.@@@@@@@@ ,@@@@@@@@@@^.@@@@@@@@@@@@
//    .@@@@.    =@@@^ =@@@^   =@@@^  ,/@@@@@@@@O =@@@^   @@@@@@@@@@@@@/  =@@@^  @@@@.   =@@@^       .@@@@   .@@@@
//    .@@@@.    =@@@^ =@@@^   =@@@^ @@@@@@@@@@@O =@@@^   @@@@@@@@@@@@/   =@@@^  @@@@.   =@@@^       .@@@@   .@@@@
//    .@@@@]]]]/@@@@^ =@@@\]]]@@@@^.@@@@]]]/@@@O =@@@\]` @@@@^           =@@@^  @@@@\]]`=@@@@`]]]]]`.@@@@   .@@@@
//    .@@@@@@@@@@@@`  .@@@@@@@@@@@^ =@@@@@@@@@@O .@@@@@^ @@@@^           =@@@^  ,@@@@@@@ \@@@@@@@@@^.@@@@   .@@@@
                                                                                                              
                                                                                                              
                                                                                                              

//------------------------双PITCH模式相关-----------------------------------------
float pid_param_backup[24];     // 单PITCH参数备份存在此。在初始化时备份
float dual_pitch_pid_param[24]=     //双PITCH参数在此调节
{-3, 0, -8, 2000, 30000, 500,
-15, -1, 0, 1800, 10000,120,
100, 0, 0, 2000, 10000,3000,
-10, 0, 0, 2000, 30000,500};
uint8_t is_use_dual_param;
// 参照一下参数设定
//   PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
//   PitchPosition(-15, -1, 0, 1800, 10000, 10, 10, 120),//(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
//   PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
//   PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),

static void copy_pid_param_to_array(float* destination,pid* source)
{
	destination[0] = source->P;
	destination[1] = source->I;
	destination[2] = source->D;
	destination[3] = source->IMax;
	destination[4] = source->PIDMax;
	destination[5] = source->I_Limited;
}
static void copy_array_to_pid_param(pid* desti,float* src)
{
	desti->P = src[0];
	desti->I = src[1];
	desti->D = src[2];
	desti->IMax = src[3];
	desti->PIDMax = src[4];
	desti->I_Limited = src[5];
}

///备份单PITCH参数到 pid_param_backup. BackupSingleMotorParam
static void copy_cloud_param_to_backup(softcloud* src)
{
    copy_pid_param_to_array(&pid_param_backup[0],src->PID_In);
	copy_pid_param_to_array(&pid_param_backup[6],src->PID_Out);
	copy_pid_param_to_array(&pid_param_backup[12],src->Gyro_PID_In);
    copy_pid_param_to_array(&pid_param_backup[18],src->Gyro_PID_Out);
}
///复制数组中的参数到云台电机PID
static void copy_array_to_cloud_param(softcloud* desti,float* src)
{
    copy_array_to_pid_param(desti->PID_In,&src[0]);
    copy_array_to_pid_param(desti->PID_Out,&src[6]);
    copy_array_to_pid_param(desti->Gyro_PID_In,&src[12]);
    copy_array_to_pid_param(desti->Gyro_PID_Out,&src[18]);
}
/**
 * @brief 进入双PITCH模式
 * 
 */
void SentryCloud::EnterModeDualPitch(void)
{
    copy_array_to_cloud_param(&PitchMotor,dual_pitch_pid_param);    //写入双PITCH参数 WriteDualMotorParam();
	is_use_dual_param = 1;
    PitchSecondMotor.cooperative = 1;       //设定副PITCH为合作模式，这样会禁用它的PID运算。
    // 它的输出值电流值会在运行时复制PitchMotor的电流输出值
}
/**
 * @brief 退出双PITCH模式，恢复单PITCH模式
 * 
 */
void SentryCloud::ExitModeDualPitch(void)
{
    copy_array_to_cloud_param(&PitchMotor,pid_param_backup);    //写入单PITCH参数 WriteDualMotorParam();
	is_use_dual_param = 0;
    PitchSecondMotor.cooperative = 0;       //关闭副PITCH为合作模式，恢复它的PID运算。
    // 但是此函数在某一PITCH掉线时才会调用。
}
/**
 * @brief 运行双PITCH模式
 * 从动PITCH电机的输出值电流值会在运行时复制PitchMotor的电流输出值
 */
void SentryCloud::RunModeDualPitch()
{
	PitchSecondMotor.cooperative =1;
//    PitchSecondMotor.TargetCurrent = PitchMotor.TargetCurrent;  // 复制电流值
//    PitchSecondMotor.InsertCurrent();   // 写入电流值
}
/// 覆写 UserProc定义用于操作电流值
void manager::UserProcess(void)
{
	if(CloudEntity.PitchSecondMotor.cooperative == 1)
	{
		CloudEntity.PitchSecondMotor.TargetCurrent = CloudEntity.PitchMotor.TargetCurrent;  // 复制电流值
		CloudEntity.PitchSecondMotor.InsertCurrent();   // 写入电流值
	}
}
/**
 * @brief PITCH模式控制。在Handle()中调用
 * 
 */
void SentryCloud::PitchModeCtrl(void)
{
	last_pitch_ctrl_mode = pitch_ctrl_mode;	// 记录旧的 PITCH模式
	
	// PITCH 模式切换
	if(PitchMotor.Is_Offline() == 0 && PitchSecondMotor.Is_Offline() == 0)
	{
		// 仅当 两个PITCH在线时采用 双PITCH模式。采用合作模式。主PITCH参数写入双PITCH模式的参数
		pitch_ctrl_mode = __cloud_dual_pitch;	
	}
	else
	{
		// 当某一PITCH离线 使用副PUTCH模式，即关闭合作模式。
		// 主PITCH参数写入独立模式的参数。两PITCH的PID独立运行。
		// 由于一个PITCH离线。
		pitch_ctrl_mode = __cloud_second_pitch;
	}
	
	// 进行模式切换及运行
	
	if(last_pitch_ctrl_mode != pitch_ctrl_mode)
	{
		switch(last_pitch_ctrl_mode)
		{
			case __cloud_dual_pitch:
				ExitModeDualPitch();
				break;
			default:
				break;
		}
		switch(pitch_ctrl_mode)
		{
			case __cloud_dual_pitch:
				EnterModeDualPitch();
				break;
			default:
				break;
		}
	}
	
	switch(pitch_ctrl_mode)
	{
		case __cloud_dual_pitch:
			RunModeDualPitch();
			break;
		case __cloud_second_pitch:
			PitchSecondMotor.cooperative = 0;
			break;
		default:
			break;

	}
	
}
/**
 * @brief 软件限位控制
 * 
 */
void SentryCloud::PitchRealAngleLimitCtrl()
{
    // if(PitchMotor.RunState == Speed_Ctl || PitchMotor.RunState == Gyro_Speed_Ctl)
    // {
        if(PitchMotor.RealAngle >= pitch_limit_max)
        {
            PitchMotor.Angle_Set(pitch_limit_max);
			PitchMotor.PID_Out->Iout = 0;
        }
        if(PitchMotor.RealAngle <= pitch_limit_min)
        {
            PitchMotor.Angle_Set(pitch_limit_min);
			PitchMotor.PID_Out->Iout = 0;
        }
    // }
    // else    // if (PitchMotor.RunState == Position_Ctl || PitchMotor.RunState == Gyro_Position_Ctl)
    // {

    // }

    if(PitchSecondMotor.cooperative == 0)
    {
        if(PitchSecondMotor.RealAngle >= pitch_limit_max)
        {
            PitchSecondMotor.Angle_Set(pitch_limit_max);
			PitchSecondMotor.PID_Out->Iout = 0;
        }
        if(PitchSecondMotor.RealAngle <= pitch_limit_min)
        {
            PitchSecondMotor.Angle_Set(pitch_limit_min);
			PitchSecondMotor.PID_Out->Iout = 0;
        }
    }
}


// 电机型号类
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508_Fric(8192, 1);


// 电机实体定义 >>>>>>>>>>>>>>>>重要<<<<<<<<<<<<<
SentryCloud CloudEntity(1, 0x206, 1, 0x205, 2, 0x206, 1, 0x204, 1, 0x203, 1, 0x201);


/// 云台物理实体类 构造与删除函数
SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
						 uint8_t pitch2nd_can_num, uint16_t pitch2nd_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)
        // 初始化各项PID参数
    : PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
	  PitchPosition(-10, -1, 0, 1800, 10000, 10, 10, 120),//(15, 1, 0, 1800, 10000, 10, 10, 120)(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
      PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),
	  Pitch2ndSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500),
	  Pitch2ndPosition(1, 0, 0, 1800, 10000, 10, 10, 120),
	  Pitch2ndGyroPosition(6, 0, 8, 2000, 30000, 10, 10, 500),
	  Pitch2ndGyroSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
      YawSpeed(20, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(0, 0,0.5, 200, 10000, 10, 2, 100),//(10, 1,0.5, 200, 10000, 10, 2, 100) (10, 0, 0, 2000, 10000, 10, 10, 3000)
      YawGyroSpeed(-15, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FeedSpeed(20, 0, 1, 1000, 7000),
      FeedPositon(0.5, 0.01, 0, 1000, 20000, 0, 200),
/*    : PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
	  PitchPosition(-15, -1, 0, 1800, 10000, 10, 10, 120),//(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
      PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),
	  Pitch2ndSpeed(6, 0, 8, 2000, 30000, 10, 10, 500),
	  Pitch2ndPosition(15, 1, 0, 1800, 10000, 10, 10, 120),
	  Pitch2ndGyroPosition(6, 0, 8, 2000, 30000, 10, 10, 500),
	  Pitch2ndGyroSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
      YawSpeed(20, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(10, 1,-0.5, 200, 10000, 10, 2, 100),//10, 0, 0, 2000, 10000, 10, 10, 3000)
      YawGyroSpeed(15, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FeedSpeed(20, 0, 1, 1000, 7000),
      FeedPositon(0.5, 0.01, 0, 1000, 20000, 0, 200),
*/
        //!!!!!!!!!!!>>>>>>>>>>>>>要调节双PITCH参数请到 dual_pitch_pid_param

        // 初始化各电机参数
	  YawMotor(yaw_can_num, yaw_can_id, 1319, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &BaseImuAngleRate[2]),      // 请注意YAW轴位置环直接采取的是底座的朝向
      PitchMotor(pitch_can_num, pitch_can_id, 52, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      PitchSecondMotor(pitch2nd_can_num, pitch2nd_can_id, 4115, &DJI_6020, &Pitch2ndSpeed, &Pitch2ndPosition, &Pitch2ndGyroSpeed, &Pitch2ndGyroPosition,&RotatedImuAngleRate[1], &RotatedImuAngle[1]),
	  FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{
	Feed2nd.Enable_Block(4000,200,5);                       // 初始化堵转检测
	PitchPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;  // 设定微分来源为陀螺仪
	Pitch2ndPosition.Custom_Diff = PitchSecondMotor.Gyro_RealSpeed;
	YawPosition.Custom_Diff = YawMotor.Gyro_RealSpeed;      // 设定微分来源为陀螺仪
	
    PitchPosition.pid_run_CallBack = pidPitchCallBack;  // 位置环PID的用户自定义回调函数。加入重力前馈函数。
    PitchGyroPosition.pid_run_CallBack = pidPitchCallBack;  //位置环PID的用户自定义回调函数。加入重力前馈函数。
	
//	PitchSecondMotor.cooperative=1;	// 设为合作模式，这样会阻断此电机的PID的运行
	
	copy_cloud_param_to_backup(&PitchMotor);				// 复制云台PITCH轴参数到备份数组。以待运行时使用。
	is_use_dual_param = 0;	// 标记未使用双PITCH参数。此数值会随着PITCH模式变化
	SetPitchRealAngleLimit(35,-60);
};









void SentryCloud::Handle()
{
    //------------------------安全模式激光灯自动关闭-------------------------
	if(Mode != save_cloud)
		LazerSwitchCmd(1);  
    else
        LazerSwitchCmd(0);  
	
    // 陀螺仪数据处理
    ImuDataProcessHandle();
    //--------------------------单/双PITCH模式控制逻辑--------------------------
    PitchModeCtrl();    
    //--------------------------------射击控制逻辑-----------------------------------
    ShootCtrl();

    // -------------------------------分模式逻辑-------------------------------
    {
        switch (Mode)   // 根据 Mode 选择模式
        {

            case absolute_gyro_cloud:
            case relative_gyro_cloud:
                CurrentCloudMode = &ModeCloudCtrlGyro;
                break;

            case absolute_cloud:
            case relative_cloud:
            default:
                CurrentCloudMode = &ModeCloudCtrlMech;
                break;

        }

        if(LastCloudMode != CurrentCloudMode)   // 如果模式更新
        {
            LastCloudMode->Exit();              // 使用模式切换用函数
            CurrentCloudMode->Enter();
        }
        CurrentCloudMode->Run();            // 正常执行模式内逻辑

        LastCloudMode = CurrentCloudMode;   //  检查模式更新
    }
    //-------------------------------PITCH软件限位----------------------------------
    PitchRealAngleLimitCtrl();

//CANSend会在主逻辑统一调用
//    manager::CANSend();


    
}
/**
 * @brief 陀螺仪数据处理
 * 
 */
void SentryCloud::ImuDataProcessHandle()
{
	if(Mode != relative_gyro_cloud && Mode != absolute_gyro_cloud)  //不在陀螺仪控制模式中时，陀螺仪角度始终跟随机械角角度（但是要旋转回陀螺仪角度）
	{
		app_imu_data.integral.Pitch = -CloudEntity.PitchMotor.RealAngle;//注意负号。
		app_imu_data.integral.Yaw = -CloudEntity.YawMotor.RealAngle;//注意负号。
	}
	//↓↓↓陀螺仪角度旋转到枪口方向↓↓↓
	RotatedImuAngle[0] = -app_imu_data.integral.Pitch;
    RotatedImuAngle[1] = app_imu_data.integral.Roll;
	RotatedImuAngle[2] = app_imu_data.integral.Yaw;
	//↓↓↓陀螺仪加速度旋转到枪口方向↓↓↓
	RotatedImuAngleRate[0] = -app_imu_data.Angle_Rate[1];
    RotatedImuAngleRate[1] = app_imu_data.Angle_Rate[0];
	RotatedImuAngleRate[2] = app_imu_data.Angle_Rate[2];
    //↓↓↓陀螺仪角速度旋转到炮塔方向，即Roll,Pitch水平，Yaw随枪口Yaw↓↓↓
    float Cp = cosf(RealPitch), Sp = sinf(RealPitch);

    BaseImuAngleRate[0] = Cp*RotatedImuAngleRate[0] + Sp*RotatedImuAngleRate[2];
    BaseImuAngleRate[1] = RotatedImuAngleRate[1];
    BaseImuAngleRate[2] = -Sp*RotatedImuAngleRate[0] + Cp*RotatedImuAngleRate[2];
    //更新云台的Yaw,Pitch角度
	RealYaw = YawMotor.RealAngle;   //就是电机的角度
	MechanicYaw = YawMotor.RealPosition*360.f/YawMotor.MotorType->max_mechanical_position;//根据机械角计算出的真实角度
	RealPitch = - PitchMotor.RealAngle;	//注意负号
}


///设定机械角控制角度
void SentryCloud::SetAngleTo(float pitch, float yaw)
{
	Mode = absolute_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
	
	if(TargetPitch > pitch_limit_max)
		TargetPitch = pitch_limit_max;
	if(TargetPitch < pitch_limit_min)
		TargetPitch = pitch_limit_min;
	
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
	PitchSecondMotor.Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Angle_Set(TargetYaw);
}
///设定陀螺仪控制角度
void SentryCloud::SetAngleTo_Gyro(float pitch, float yaw)
{
	Mode = absolute_gyro_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Gyro_Angle_Set(-TargetPitch);
	PitchSecondMotor.Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Gyro_Angle_Set(TargetYaw);
}


/**
 * @brief 根据所在角度，切换角度控制位置环反馈源的逻辑。
 * 
 * @param     current_pitch     现在的俯仰角
 * @param     current_yaw       现在的航向角
 * @return enum _cloud_ctrl_mode 
 */
static enum _cloud_ctrl_mode decide_mode_by_angle(float current_pitch, float current_yaw)
{
    float mechanic_pitch,mechanic_yaw;

    mechanic_pitch = app_math_fLimitPeriod(current_pitch,180.0f,-180.0f);
    mechanic_yaw = app_math_fLimitPeriod(current_yaw,180.0f,-180.0f);

    // if  // 转换为陀螺仪模式的条件：YAW行至会抖动的角度
    // ( 
    //     IS_IN_INTERVAL(mechanic_yaw,30,40)
    // )
    // {return absolute_gyro_cloud;}

    return absolute_cloud;
}


/**
 * @brief 通用的，可调模式的控制角度。包含【自适应选择反馈】逻辑，已解决抖动问题
 * 
 * @param     pitch 输入的俯仰角
 * @param     yaw   输入的航向角
 * @param     mode  选中的反馈模式，取值见 @see enum _cloud_ctrl_mode
 */
void SentryCloud::SenAngleTo_Generic(float pitch, float yaw, enum _cloud_ctrl_mode mode)
{
    switch(mode)
	{
		case relative_auto_cloud:	// xx_auto 模式下会自动切换 机械/陀螺仪 获取最佳控制方式
			pitch	+=	RealPitch;
			yaw		+=	RealYaw;
		case absolute_auto_cloud:
			{

                _cloud_ctrl_mode auto_decided_mode = decide_mode_by_angle(RealPitch, RealYaw);
                
                SenAngleTo_Generic(pitch,yaw,auto_decided_mode);

            }
			break;
		
		case save_cloud:
			Safe_Set();
			break;
			
		case absolute_cloud:
			SetAngleTo(pitch,yaw);
			break;
			
		case relative_cloud:
			SetAngleTo(pitch+RealPitch, yaw+RealYaw);
			break;
			
		case absolute_gyro_cloud:
			SetAngleTo_Gyro(pitch,yaw);
			break;
			
		case relative_gyro_cloud:
			SetAngleTo_Gyro(pitch+RealPitch, yaw+RealYaw);
			
		default:
			Safe_Set();
			break;
	}
}


/**
 * @brief 启动射击
 * 
 * @param     bullet_speed  出趟弹速。目前此参数控制的是摩擦轮电机的转速
 * @param     fire_freq     发射频率。这个控制。
 * @param     Shoot_mode    射击模式。这个保留备用。
 */
void SentryCloud::Shoot(float bullet_speed, uint8_t fire_freq, uint8_t Shoot_mode)
{
	Shoot_Speed = bullet_speed;     // 设定摩擦轮速率

	ShooterSwitchCmd(1);            // 启动摩擦轮和射击许可

	if(fire_freq!=0)
    {
        Feed_Free_Once_Set(60000/fire_freq,1);
    }
    else
    {
        Feed_Safe_Set();
    }
    
}


///安全模式
void SentryCloud::Safe_Set()
{
	Mode = save_cloud;
    YawMotor.Speed_Set(0);
//    PitchMotor.Safe_Set();
	PitchMotor.Speed_Set(0);
    PitchSecondMotor.Speed_Set(0);
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
//    manager::CANSend();
	LazerSwitchCmd(0);
}


//下面三个函数为拨弹电机函数的包装。需要feed_is_permitted==1才能运行供弹电机
 /**
  * @brief      经包装的连续拨弹函数
  * @param     FreeSpeed 设定拨弹轮速度
  */
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
        CloudEntity.Feed2nd.Safe_Set();
	}
	else
	{
		feed_is_permitted=1;
        FricLeftMotor.Speed_Set(-Shoot_Speed);
        FricRightMotor.Speed_Set(Shoot_Speed);
	}
}

void SentryCloud::SetPitchRealAngleLimit(float max, float min)
{
    if(max < min)
    {
        pitch_limit_max = min;
        pitch_limit_min = max;
    }
    else
    {
        pitch_limit_max = max;
        pitch_limit_min = min;
    }
}


/**
 * @brief PITCH运行回调函数。重力前馈。
 * 
 */
void pidPitchCallBack(pid* self)
{
    self->PIDout+=CloudEntity.gravity_feedforward(CloudEntity.RealPitch);
}








                                                                                                                                                                                                                                                                                 
//                           *** **********]]]]************                ***     ****************                                     
//                           *** **    * **@@@@****     ***                *           ** *=@@@^***                                     
// * * *** ****        ****   ******* *****@@@@@@****************** * ** **** *************=@@@^****  *                                 
//  *  ********        **********=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^***                                     
//    */@@@^**************@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^*****        * *** ******             
//  * *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO@@@@@@@@@@@@@@@@@^************************************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^*,*@@@@@@@@@@@@@@@@@@@@@****************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@****************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^**************************************
//     @@@@@@@@@@@@@@[\[[[[[[[[[[[[[[[[[[[\@@@@@@@@@@@@@@@@@@@[\/[[[[[[[[[[[[[[[[[[[[[[[/[[[********************************************
//     @@@@@@@@@@@@@@********************@@@@@@@@@@@@@@@@@@@@@**  *********             ************************************************
//     @@@@@@@@@@@@[`******************@@@@@@@@@@@@@@@@@@@@@@@*****                     ************************************************
// *  *@@@@@@@@@@@@******************=@@@@@@@@@@@***@@@@@@@@@@**************************************************************************
//    *@@@@@@@@@@******************=@@@@@@@@@*********@@@@@@@@@@************************************************************************
//  * *@@@@@@@@^*****************=@@@@@@@@@^**********@@@@@@@@@@,***********************************************************************
//    *\@@@@@^*****  **********=@@@@@@@@@@@^************@@@@@@@@@@**********************************************************************
//     **    ****    ************=@@@@@@@^*********** **@@@@@@@@************************************************************************
//        *          **********************  ************,@@`******      ***************************************************************

//------------------------------------射击控制相关-------------------------------------------
/**
 * @brief 云台内部 射击控制逻辑
 */
void SentryCloud::ShootCtrl()
{
	// if(REMAIN_HEAT<ONE_SHOT_HEAT)
    // {
    //     feed_is_permitted =0;
    // }


// --------------------------------射击许可开关----------------------------
    if(feed_is_permitted != 1)   //检查射击许可，不许可 则回到安全
	{
		Feed2nd.Safe_Set();
	}
    if(fric_power_permitted != 1)   // 检车摩擦轮许可
    {
        FricLeftMotor.Safe_Set();
		FricRightMotor.Safe_Set();
    }

    shoot_flag = Feed2nd.feed_mode; //写入射击状态为拨弹电机状态

}

/**
 * @brief       读取PID参数的回调函数。由外面实现
 * 
 * @param     pid_id    PID编号
 * @param     p         读取后写入P的地址
 * @param     i         读取后写入I的地址
 * @param     d         读取后写入D的地址
 * @return 状态值。正常返回HAL_OK, 异常返回HAL_ERROR 
 */
HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d)
{
    switch (pid_id)
    {
    case __cloud_pitch_position_pid:
        *p = CloudEntity.PitchPosition.P;
		*i = CloudEntity.PitchPosition.I;
		*d = CloudEntity.PitchPosition.D;
        break;
    case __cloud_yaw_position_pid:
        *p = CloudEntity.YawPosition.P;
		*i = CloudEntity.YawPosition.I;
		*d = CloudEntity.YawPosition.D;
        break;
    case __cloud_pitch_speed_pid:
        *p = CloudEntity.PitchSpeed.P;
		*i = CloudEntity.PitchSpeed.I;
		*d = CloudEntity.PitchSpeed.D;
        break;
    case __cloud_yaw_speed_pid:
        *p = CloudEntity.YawSpeed.P;
		*i = CloudEntity.YawSpeed.I;
		*d = CloudEntity.YawSpeed.D;	
        break;
    default:
        break;
    }
	return HAL_OK;
}

/**
 * @brief 实现触发射击 回调函数
 * 
 * @param     bullet_speed  子弹飞行速度。实际是云台摩擦轮电机
 * @param     fire_freq     子弹射击频率，单位是RPM。实现上是把云台供弹暂停时间改为 60*1000/fire_freq
 * @param     shoot_mode    子弹射击模式。保留备用。
 */
void CMD_SHOOT_ExecuteCallback(float bullet_speed, uint8_t fire_freq, uint8_t shoot_mode){
	CloudEntity.Shoot(bullet_speed,fire_freq,shoot_mode);
}












































































 

