/**
 * @file SentryCloud.hpp
 * @brief    哨兵上云台
 * @details     Encoding - UTF-8
 * @author   ThunderDoge
 * @date     2020-5-23
 * @version  v2.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 * 
 * v1.0 2020-4-15   发布1.0可用版本
 * v2.0 2020-5-23   重构计划"Clean Cloud"
 */
#include "SentryCloud.hpp"

// 电机型号
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508_Fric(8192, 1);

// 电机实体定义 
SentryCloud CloudEntity(1, 0x206, 1, 0x205, 2, 0x206, 1, 0x204, 1, 0x203, 1, 0x201);

/**
 * @brief 云台物理实体类 构造 Construct a new Sentry Cloud:: Sentry Cloud object
 * 
 * @param     yaw_can_num           CAN号,1或2,对应CAN1和CAN2
 * @param     yaw_can_id            电机CAN上ID. 大疆电机编码从0x201~0x20B, 下同
 * @param     pitch_can_num         
 * @param     pitch_can_id 
 * @param     pitch2nd_can_num 
 * @param     pitch2nd_can_id 
 * @param     fric_l_can_num 
 * @param     fric_l_can_id 
 * @param     fric_r_can_num 
 * @param     fric_r_can_id 
 * @param     feed_can_num 
 * @param     feed_can_id 
 */
SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
						 uint8_t pitch2nd_can_num, uint16_t pitch2nd_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)
        // 初始化各项PID参数
    : PitchSpeed(-6, 0, -8, 2001, 30000, 10, 10, 500), 
	  PitchPosition(-30, -1, 0, 3001, 10000, 10, 10, 200),//(15, 1, 0, 1800, 10000, 10, 10, 120)(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      PitchGyroSpeed(0, 0, 0, 2011, 30000, 10, 10, 500),
      PitchGyroPosition(0, 0, 0, 2011, 10000, 10, 10, 3000),
	  
	  Pitch2ndSpeed(-6, 0, -8, 2002, 30000, 10, 10, 500),
	  Pitch2ndPosition(-30, -1, 0, 1802, 10000, 10, 10, 120),
	  Pitch2ndGyroSpeed(0, 0, 0, 2000, 30000, 10, 10, 500),
	  Pitch2ndGyroPosition(0, 0, 8, 2000, 30000, 10, 10, 500),

	  DualSpeed(-3, 0, -8, 2001, 30000, 10, 10, 500), 
	  DualPosition(-30, -1, -0.5, 3001, 10000, 10, 10, 200),//(15, 1, 0, 1800, 10000, 10, 10, 120)(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      DualGyroSpeed(0, 0, 0, 2011, 30000, 10, 10, 500),
      DualGyroPosition(0, 0, 0, 2011, 10000, 10, 10, 3000),

      YawSpeed(20, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(20, 2,-0.5, 300, 10000, 10, 2, 100),//(10, 1,0.5, 200, 10000, 10, 2, 100) (10, 0, 0, 2000, 10000, 10, 10, 3000)
      YawGyroSpeed(-15, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
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
        // 初始化各电机参数
	  YawMotor(yaw_can_num, yaw_can_id, 4086, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &BaseImuAngleRate[2]),      // 请注意YAW轴位置环直接采取的是底座的朝向
      PitchMotor(pitch_can_num, pitch_can_id, 8188, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      Pitch2ndMotor(pitch2nd_can_num, pitch2nd_can_id, 4085, &DJI_6020, &Pitch2ndSpeed, &Pitch2ndPosition, &Pitch2ndGyroSpeed, &Pitch2ndGyroPosition,&RotatedImuAngleRate[1], &RotatedImuAngle[1]),
	  FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{
    // 初始化堵转检测
	Feed2nd.Enable_Block(5000,200,5);     

    // 设定位置环微分来源为陀螺仪
	PitchPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;  
	Pitch2ndPosition.Custom_Diff = Pitch2ndMotor.Gyro_RealSpeed;
	YawPosition.Custom_Diff = YawMotor.Gyro_RealSpeed;
	DualPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;

    // 设定软件限位
	SetPitchRealAngleLimit(35,-60);
};
/**
 * @brief 云台自动控制托管
 * 
 */
void SentryCloud::Handle()
{	
    /* 陀螺仪数据处理 */
    ImuDataProcessHandle();
    // 单/双PITCH模式控制逻辑
    PitchModeCtrl();

    // 云台模式控制逻辑
    CloudModeCtrl();

    // 射击控制逻辑
    ShootCtrl();
    // PITCH软件限位
    PitchRealAngleLimitCtrl();

// CANSend会在主逻辑统一调用
    // manager::CANSend(); 
}

/**
 * @brief 安全模式
 * 
 */
void SentryCloud::Safe_Set()
{
	CloudMode = save_cloud;
    YawMotor.Speed_Set(0);
	PitchMotor.Speed_Set(0);
    Pitch2ndMotor.Speed_Set(0);
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
	LazerSwitchCmd(0);
    ShooterSwitchCmd(0);
}
void SentryCloud::Safe_Set_NoMode()
{
    YawMotor.Speed_Set(0);
	PitchMotor.Speed_Set(0);
    Pitch2ndMotor.Speed_Set(0);
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
	LazerSwitchCmd(0);
    ShooterSwitchCmd(0);
}
/**
 * @brief 设定机械角控制角度
 * 这个函数会自动把云台模式设定成绝对角度-陀螺仪控制
 * 如果你没有别的需求推荐直接使用这个函数
 * @param     pitch 
 * @param     yaw 
 */
void SentryCloud::SetAngleTo(float pitch, float yaw)
{
	CloudMode = absolute_cloud;              // 设定模式
    LastControlTick = HAL_GetTick();    // 更新时间戳
	
	if(pitch > pitch_limit_max)
		pitch = pitch_limit_max;
	if(pitch < pitch_limit_min)
		pitch = pitch_limit_min;

    TargetPitch = pitch;
    TargetYaw = yaw;
	
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
	Pitch2ndMotor.Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Angle_Set(TargetYaw);
}
/**
 * @brief 设定陀螺仪控制角度，并且设定模式
 * 这个函数会自动把云台模式设定成绝对角度-陀螺仪控制
 * 如果你没有别的需求推荐直接使用这个函数
 * @param     pitch 
 * @param     yaw 
 */
void SentryCloud::SetAngleTo_Gyro(float pitch, float yaw)
{
	CloudMode = absolute_gyro_cloud;         // 设定模式
    LastControlTick = HAL_GetTick();    // 更新时间戳
	
	if(pitch > pitch_limit_max)
		pitch = pitch_limit_max;
	if(pitch < pitch_limit_min)
		pitch = pitch_limit_min;

    TargetPitch = pitch;
    TargetYaw = yaw;

    PitchMotor.Gyro_Angle_Set(-TargetPitch);
	Pitch2ndMotor.Gyro_Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Gyro_Angle_Set(TargetYaw);
}
/**
 * @brief 设定角度 - 自动选择控制
 * 
 * @param     pitch 
 * @param     yaw 
 */
void SentryCloud::SetAngleTo_Auto(float pitch, float yaw)
{
	CloudMode = auto_cloud;         // 设定模式为自动选择控制模式
    LastControlTick = HAL_GetTick();    // 更新时间戳
	
	if(pitch > pitch_limit_max)
		pitch = pitch_limit_max;
	if(pitch < pitch_limit_min)
		pitch = pitch_limit_min;

    TargetPitch = pitch;
    TargetYaw = yaw;

    // 如果是使用auto的话控制模式将会在 Handle() 中进行

    PitchMotor.Gyro_Angle_Set(-TargetPitch);
	Pitch2ndMotor.Gyro_Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    // YAW电机在此不进行设定，在YawMeGyModeCtrl()中会进行设定
    // YawMotor.Gyro_Angle_Set(TargetYaw);
}

/**
 * @brief 仅设定陀螺仪控制角度 - 仅供内部调用
 * 不会更改模式
 * @param     pitch 
 * @param     yaw 
 */
void SentryCloud::SetAngleTo_NoMode(float pitch, float yaw)
{
	if(pitch > pitch_limit_max)
		pitch = pitch_limit_max;
	if(pitch < pitch_limit_min)
		pitch = pitch_limit_min;

    TargetPitch = pitch;
    TargetYaw = yaw;
	
    PitchMotor.Angle_Set(-TargetPitch);	//注意负号
	Pitch2ndMotor.Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Angle_Set(TargetYaw);
}
/**
 * @brief 设定机械角控制角度 - 仅供内部调用
 * 不会更改模式
 * @param     pitch 
 * @param     yaw 
 */
void SentryCloud::SetAngleTo_Gyro_NoMode(float pitch, float yaw)
{
	if(pitch > pitch_limit_max)
		pitch = pitch_limit_max;
	if(pitch < pitch_limit_min)
		pitch = pitch_limit_min;

    TargetPitch = pitch;
    TargetYaw = yaw;

    PitchMotor.Gyro_Angle_Set(-TargetPitch);
	Pitch2ndMotor.Gyro_Angle_Set(-TargetPitch);	// 为副PITCH电机设置相同的。如果是双PITCH模式会自动覆盖。
    YawMotor.Gyro_Angle_Set(TargetYaw);
}

/**
 * @brief 
 * 
 * @param     newCloudMode 
 */
void SentryCloud::SetCloudMode(CloudMode_t newCloudMode)
{
    CloudMode = newCloudMode;
}


/**
 * @brief 启动射击
 * 
 * @param     bullet_speed  出趟弹速。目前此参数控制的是摩擦轮电机的转速
 * @param     fire_freq     发射频率。这个控制。
 * @param     Shoot_mode    射击模式。这个保留备用。
 */
void SentryCloud::Shoot(float bullet_speed, uint32_t fire_freq, ShootModeEnum_t shoot_mode)
{
	Shoot_Speed = bullet_speed;     // 设定摩擦轮速率

    ShootMode = shoot_mode;         

	ShooterSwitchCmd(1);            // 启动摩擦轮和射击许可

    feed_trig = 1;

	if(fire_freq!=0)
    {
        Feed2nd.Free_Once_Set((uint32_t)(60000/fire_freq),&feed_trig);
    }
    else
    {
        Feed2nd.Safe_Set();
    }
    
}

/**
 * @brief 激光灯开关
 * 
 * @param     NewState 
 */
void SentryCloud::LazerSwitchCmd( int NewState )
{
	if(NewState == 0)
	{
		HAL_GPIO_WritePin(LAZER_GPIO_Port,LAZER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LAZER_GPIO_Port,LAZER_Pin,GPIO_PIN_SET);
	}
}

/**
 * @brief 摩擦轮开关
 * 
 * @param     NewState 
 */
void SentryCloud::ShooterSwitchCmd(int NewState )
{
	if(NewState == 0)
	{
        FricLeftMotor.Safe_Set();
        FricRightMotor.Safe_Set();
        CloudEntity.Feed2nd.Safe_Set();
	}
	else
	{
        FricLeftMotor.Speed_Set(-Shoot_Speed);
        FricRightMotor.Speed_Set(Shoot_Speed);
	}
}
/**
 * @brief 云台模式控制逻辑
 * 
 */
void SentryCloud::CloudModeCtrl()
{
    switch (CloudMode)
    {
    case absolute_cloud:
    case absolute_gyro_cloud:
    case relative_cloud:
    case speed_cloud:
        // 正常控制模式的现在不需过此处理
        break;

    case hold_cloud:  
        // 设定角度为当前角度
        SetAngleTo_NoMode(TargetPitch, TargetYaw);
        break;

    case save_cloud:
        Safe_Set_NoMode();
    
    case default_cloud_mode:
    default:
        CloudMode = default_cloud_mode;
        Safe_Set_NoMode();
        break;
    }
}
/**
 * @brief 设定云台pitch软件限位
 * 
 * @param     max 
 * @param     min 
 */
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
 * @brief 软件限位控制
 * 
 */
void SentryCloud::PitchRealAngleLimitCtrl()
{
    // 复制旧的pitch_exceed_flag
    memcpy(pitch_last_exceed_flag,pitch_exceed_flag,sizeof(pitch_last_exceed_flag));

    // 速度控制情况下，只要 RealAngle 超过软件限，直接 Angle_Set(Position_Ctl事模式)到软件限
    // 并且设定 pitch_exceed_flag[xx] = 1; 这会导致IMAX置零 见下面。
    if(PitchMotor.RunState == Speed_Ctl || PitchMotor.RunState == Gyro_Speed_Ctl)
    {
        if(PitchMotor.RealAngle > pitch_limit_max)
        {
            PitchMotor.Angle_Set(pitch_limit_max);
            pitch_exceed_flag[0] = 1;
        }
        else if(PitchMotor.RealAngle < pitch_limit_min)
        {
            PitchMotor.Angle_Set(pitch_limit_min);
            pitch_exceed_flag[0] = 1;
        }
        else
        {
            pitch_exceed_flag[0] = 0;
        }
    }
    // 位置控制情况下，仅在 RealAngle 和 TargetAngle 都超过软件限 会 设定 pitch_exceed_flag
    else if(PitchMotor.RunState == Position_Ctl || PitchMotor.RunState == Gyro_Position_Ctl)
    {
        if(PitchMotor.RealAngle > pitch_limit_max && PitchMotor.TargetAngle > pitch_limit_max)
        {
            PitchMotor.Angle_Set(pitch_limit_max);
            pitch_exceed_flag[0] = 1;
        }
        else if(PitchMotor.RealAngle < pitch_limit_min && PitchMotor.TargetAngle < pitch_limit_min)
        {
            PitchMotor.Angle_Set(pitch_limit_min);
            pitch_exceed_flag[0] = 1;
        }
        else
        {
            pitch_exceed_flag[0] = 0;
        }
    }

    if(Pitch2ndMotor.cooperative == 0)
    {
        if(Pitch2ndMotor.RealAngle >= pitch_limit_max)
        {
            Pitch2ndMotor.Angle_Set(pitch_limit_max);
            pitch_exceed_flag[1] = 1;
        }
        else if(Pitch2ndMotor.RealAngle <= pitch_limit_min)
        {
            Pitch2ndMotor.Angle_Set(pitch_limit_min);
            pitch_exceed_flag[1] = 1;
        }
        else
        {
            pitch_exceed_flag[1] = 0;
        }
    }
	
    // 超限保存 IMAX，并且写入IMAX=0，关闭积分作用以避免抖动
    // 回到限内 从保存变量回复 IMAX
	if(CloudEntity.pitch_exceed_flag[0] && !CloudEntity.pitch_last_exceed_flag[0])
	{
		CloudEntity.pitch_IMax_save[0] = CloudEntity.PitchMotor.PID_Out->IMax;
		CloudEntity.PitchMotor.PID_Out->IMax = 0;
	}
	if(!CloudEntity.pitch_exceed_flag[0] && CloudEntity.pitch_last_exceed_flag[0])
	{
		CloudEntity.PitchMotor.PID_Out->IMax = CloudEntity.pitch_IMax_save[0];
	}
	
	if(CloudEntity.pitch_exceed_flag[1] && !CloudEntity.pitch_last_exceed_flag[1])
	{
		CloudEntity.pitch_IMax_save[1] = CloudEntity.Pitch2ndMotor.PID_Out->IMax;
		CloudEntity.Pitch2ndMotor.PID_Out->IMax = 1;
	}
	if(!CloudEntity.pitch_exceed_flag[1] && CloudEntity.pitch_last_exceed_flag[1])
	{
		CloudEntity.Pitch2ndMotor.PID_Out->IMax = CloudEntity.pitch_IMax_save[1];
	}

}

/**
 * @brief PITCH模式控制。在Handle()中调用
 */
void SentryCloud::PitchModeCtrl(void)
{	
	// PITCH 模式切换
	if(PitchMotor.Is_Offline() == 0 && Pitch2ndMotor.Is_Offline() == 0)
	{
		// 仅当 两个PITCH在线时采用 双PITCH模式。采用合作模式。主PITCH参数写入双PITCH模式的参数
		pitch_ctrl_mode = __cloud_dual_pitch;	
	}
	else
	{
		// 当某一PITCH离线 使用副PITCH模式，即关闭合作模式。
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
                /* 退出双PITCH模式 */
                
                PitchMotor.Pid_Select(&PitchSpeed,&PitchPosition);  // PID切回单pitch PID
                PitchMotor.Gyro_Pid_Select(&PitchGyroSpeed,&PitchGyroPosition);

                PitchSpeed.Iout =  DualSpeed.Iout*2;        // 装载双pitch的Iout以避免突变
                PitchPosition.Iout = DualPosition.Iout*2;   // *2是因为单pitch输出减少一半
                PitchGyroSpeed.Iout =  DualGyroSpeed.Iout*2;
                PitchGyroPosition.Iout = DualGyroPosition.Iout*2;

                Pitch2ndSpeed.Iout =  DualSpeed.Iout*2;     // 两边都要装载哦
                Pitch2ndPosition.Iout = DualPosition.Iout*2;   
                Pitch2ndGyroSpeed.Iout =  DualGyroSpeed.Iout*2;     
                Pitch2ndGyroPosition.Iout = DualGyroPosition.Iout*2;   
                
                Pitch2ndMotor.cooperative = 0;       //关闭副PITCH为合作模式，恢复它的独立PID运算和执行。
                // 这样不会导致两个pitch冲突，因为只有一个电机失效时会切到单pitch 模式
				break;
			default:
				break;
		}
		switch(pitch_ctrl_mode)
		{
			case __cloud_dual_pitch:
                /* 载入双pitch模式 */
                // copy_array_to_cloud_param(&PitchMotor,dual_pitch_pid_param);    //写入双PITCH参数 WriteDualMotorParam();
                // is_use_dual_param = 1;
                PitchMotor.Pid_Select(&DualSpeed,&DualPosition);  // PID切回到双pitch PID
                PitchMotor.Gyro_Pid_Select(&DualGyroSpeed,&DualGyroPosition);

                // 装载双pitch的Iout以避免突变
                // *2是因为单pitch输出减少一半
                DualSpeed.Iout =  PitchSpeed.Iout/2;
                DualPosition.Iout = PitchPosition.Iout/2;
                DualGyroSpeed.Iout =  PitchGyroSpeed.Iout/2;
                DualGyroPosition.Iout = PitchGyroPosition.Iout/2;

                Pitch2ndMotor.cooperative = 1;       //设定副PITCH为合作模式，这样会禁用它的PID运算。
				break;
			default:
				break;
		}
	}
	
    // 运行pitch模式
	switch(pitch_ctrl_mode) //
	{
		case __cloud_dual_pitch:
            Pitch2ndMotor.cooperative =1;
			break;
		default:
			Pitch2ndMotor.cooperative = 0;
			break;
	}

    last_pitch_ctrl_mode = pitch_ctrl_mode;	// 记录旧的 PITCH模式

}
/**
 * @brief 更新YAW轴控制模式建议
 * 即更新 Yaw_MeGy_Advice变量。在auto模式下这变量会影响控制模式
 * 为了使用auto模式你应当使用SetAngleTo_Auto函数
 */
void SentryCloud::YawMeGyModeCtrl(void)
{
    if(MechanicYaw == 180.0f)
    {
        Yaw_MeGy_Advice = GyroCtrl;
    }
    else
    {
        Yaw_MeGy_Advice = MechCtrl;
    }
    
    if(CloudMode = auto_cloud)
    {
        switch (Yaw_MeGy_Advice)
        {
        case GyroCtrl:
            YawMotor.Gyro_Angle_Set(TargetYaw);
            break;
        case MechCtrl:
            break;
            YawMotor.Angle_Set(TargetYaw);
        default:
            Yaw_MeGy_Advice = NoneCtrl;
            YawMotor.Safe_Set();
            break;
        }
    }
}
/**
 * @brief 云台内部 射击控制逻辑
 */
void SentryCloud::ShootCtrl()
{
    if( !FricLeftMotor.Is_Offline() && !FricLeftMotor.Is_Offline() )
    {
        if(fabs((float)FricLeftMotor.RealSpeed) > MINIMAL_FRIC_SPD_BAN_FEED && fabs((float)FricRightMotor.RealSpeed) > MINIMAL_FRIC_SPD_BAN_FEED)
            IsSlowFricBanFeed = 0;
        else
            IsSlowFricBanFeed = 1;
    }
    else
        IsSlowFricBanFeed = 1;

    if(IsSlowFricBanFeed)   
	{
		Feed2nd.Safe_Set();
	}
}
/**
 * @brief 实现紧急开关的功能
 * 
 */
void SentryCloud::RedButtonEffectCtrl(void)
{
    if(RED_BUTTON_HaltAllPids)  // 停止所有PID
    {
        YawSpeed.PIDMax = 0;
        YawGyroSpeed.PIDMax = 0;
        PitchSpeed.PIDMax = 0;
        PitchGyroSpeed.PIDMax = 0;
        Pitch2ndSpeed.PIDMax = 0;
        Pitch2ndGyroSpeed.PIDMax = 0;
        DualSpeed.PIDMax = 0;
        DualGyroSpeed.PIDMax = 0;
        FricLeftSpeed.PIDMax = 0;
        FricRightSpeed.PIDMax = 0;
        FeedSpeed.PIDMax = 0;

        RED_BUTTON_HaltAllPids = 0;
    }
}
/**
 * @brief 陀螺仪数据处理
 * 
 */
void SentryCloud::ImuDataProcessHandle()
{
	if(CloudMode != absolute_gyro_cloud)  //不在陀螺仪控制模式中时，陀螺仪角度始终跟随机械角角度（但是要旋转回陀螺仪角度）
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


/// 覆写 UserProc定义用于操作电流值
void manager::UserProcess(void)
{
	if(CloudEntity.Pitch2ndMotor.cooperative == 1)
	{
		CloudEntity.Pitch2ndMotor.TargetCurrent = CloudEntity.PitchMotor.TargetCurrent;  // 复制电流值
		CloudEntity.Pitch2ndMotor.InsertCurrent();   // 写入电流值
	}
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

