/**
  * @file   SentryCloudLogic.cpp
  * @brief    哨兵控制逻辑
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudLogic.hpp"


//模式定义                      进入函数                   运行时函数           退出函数
app_Mode ModeManualChassis      (NULL,                  ManualChassis,      NULL);
app_Mode ModeManualShoot        (ManualShootEnter,      ManualShoot,        nullptr);
app_Mode ModeManualFeed         (nullptr,               ManualFeed,         nullptr);
app_Mode ModeManualShootGyro    (ManualShoot_Gyro_Enter,ManualShoot_Gyro,   nullptr);
app_Mode ModeVisionControl      (nullptr,               VisionControl,      nullptr);
app_Mode ModeAutoMove           (nullptr,               nullptr,            nullptr);
app_Mode ModeGlobalSafe         (nullptr,               GlobalSafe,         nullptr);
app_Mode ModeVisionFeed         (nullptr,               VisionFeed,         nullptr);
app_Mode ModeCalibration        (nullptr,               HardCalibration,    nullptr);

// 初始化模式指针
app_Mode *LastMode = &ModeGlobalSafe;
app_Mode *CurrentMode = &ModeGlobalSafe;

extern sentry_vision_data VisionRx, VisionTx;   // 视觉接收/发送缓存
int t=4;
float p=0,y=0;
// 硬件校准模式控制变量
uint8_t inside_joystick,last_joystick,trig_calibration;
uint32_t inside_time;
/**
  * @brief  模式选择函数，控制逻辑源于此
  */
void ModeSelect(void)
{
    int mode = bsp_dbus_Data.S1 * 10 + bsp_dbus_Data.S2;
    // LastGlobalMode = GlobalMode;
    LastMode = CurrentMode;

    switch (mode)
    {
    case 32: //中-下：手动控底盘
        CurrentMode = &ModeManualChassis;
        break;
    case 12: //上-下：手动控云台
        CurrentMode = &ModeManualShoot;
        break;
    case 33: //双中：视觉控制云台转动
        CurrentMode = &ModeVisionControl;
        break;
    case 31: //中-上：视觉控制云台，手动供弹射击（右摇杆右拨为扳机）
        CurrentMode = &ModeVisionFeed;
        break;
    case 11: //上-上：遥控器测试云台 陀螺仪模式【未完成】
        CurrentMode = &ModeManualShootGyro;
        break;
    case 13: //上-中：手动控云台，手动供弹射击（右摇杆右拨为扳机）
        CurrentMode = &ModeManualFeed;
        break;
    case 22: //双下：安全。双杆朝内可进入陀螺仪校准
        if(bsp_dbus_Data.CH_0 < -500 && bsp_dbus_Data.CH_2>500 && !app_check_IsOffline(id_Dbus))
        {
            inside_joystick = 1;
        }
        else
        {
            inside_joystick = 0;
            trig_calibration = 0;
        }
        if(inside_joystick && !last_joystick)
        {
            inside_time = HAL_GetTick();
        }
        if(inside_joystick && (HAL_GetTick() - inside_time > 1000) )
        {
            if(!trig_calibration)
            {
                trig_calibration = 1;
                CurrentMode = & ModeCalibration;
				break;
            }
        }
        last_joystick = inside_joystick;
    case 23: //下中
    case 21: //下上
    default: //默认
        CurrentMode = &ModeGlobalSafe;
        break;
    }
	if(app_check_IsOffline(id_Dbus))
	{
		if(t==0)
		{
			CurrentMode = &ModeGlobalSafe;
		}
		if(t==1)
		{
			CurrentMode =&ModeManualShoot;
			CloudEntity.SetAngleTo(p,y);
			CloudEntity.ShooterSwitchCmd(0);
		}
		if(t==2)
		{
			CurrentMode = &ModeManualFeed;
			bsp_dbus_Data.CH_0=0;
		}
		if(t==3)
		{
			CurrentMode = &ModeManualFeed;
			bsp_dbus_Data.CH_0=201;
		}
		if(t==4)
		{
			CurrentMode = &ModeVisionControl;
		}
	}
	
    if (LastMode != CurrentMode)
    {
        LastMode->Exit();
        CurrentMode->Enter();
    }
    CurrentMode->Run();
}


/**
  * @brief  视觉控制云台
  */
void VisionControl(void)
{
    //云台指令处理

    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        CloudEntity.SetAngleTo_Auto(CloudEntity.RealPitch + VisionRx.Pitch,
                               CloudEntity.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        CloudEntity.SetAngleTo_Auto(VisionRx.Pitch, VisionRx.Yaw);
        break;
    case speed_cloud:
        CloudEntity.PitchMotor.Angle_Set(CloudEntity.RealPitch + VisionRx.Pitch);
        CloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
        break;
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = hold_cloud; //处理完成标志。因为一个命令只会处理一次，处理后置0
	CloudEntity.LazerSwitchCmd(1);

}
/**
  * @brief  遥控器测试云台
  */
const float dbus_rate = -0.00005;
void ManualShootEnter()
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.CloudMode = absolute_cloud; //视为绝对角控制
}
uint8_t test_use_auto=1;    /// 使用SetAngleTo_Auto标志位
void ManualShoot()
{
    //在这之前应当已调用ManualShootEnter()函数
    //目标pitch和yaw受灵敏度dbus_rate*摇杆值控制
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_3 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_2 * dbus_rate;
	if(test_use_auto)
	{
		CloudEntity.SetAngleTo_Auto(up_pitch, up_yaw);
	}
	else
	{
		CloudEntity.SetAngleTo(up_pitch, up_yaw);
	}
	CloudEntity.LazerSwitchCmd(1);
	CloudEntity.ShooterSwitchCmd(0);   
//    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
//                  (float)(bsp_dbus_Data.CH_2 * 10000.0f / 660.0f),
//                  0);
}
/**
  * @brief  遥控器测试云台，陀螺仪模式
  */
void ManualShoot_Gyro()
{
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_3 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_2 * dbus_rate;
    CloudEntity.SetAngleTo_Gyro(up_pitch, up_yaw);
	CloudEntity.LazerSwitchCmd(1);
	CloudEntity.ShooterSwitchCmd(0);   
}
void ManualShoot_Gyro_Enter()
{
    // CloudEntity.TargetPitch = CloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    // CloudEntity.TargetYaw = CloudEntity.RealYaw;
    // CloudEntity.CloudMode = absolute_gyro_cloud; //视为绝对角控制
    CloudEntity.SetAngleTo_Gyro(CloudEntity.RealPitch,CloudEntity.RealYaw);
}
/**
  * @brief  遥控器测试底盘
  */
void ManualChassis() //手动底盘
{
	CloudEntity.ShooterSwitchCmd(0);   
	CloudEntity.LazerSwitchCmd(0);

    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                  (float)(bsp_dbus_Data.CH_0 * 10000.0f / 660.0f),
                  0);
}
void ManualChassisEnter(){
    
}

/**
  * @brief  遥控器测试拨弹
  */
float feed_gap=100;
void ManualFeed()
{
    // SentryCanSend(&CAN_INTERBOARD, UP_FEED, feed_speed, 0.0f);
    // CloudEntity.FricLeftMotor.Speed_Set(-Shoot_Speed);
    // CloudEntity.FricRightMotor.Speed_Set(Shoot_Speed);
    // CloudEntity.Feed2nd.Free_Once_Set(100, (bsp_dbus_Data.CH_0 > 200));

    CloudEntity.ShooterSwitchCmd(1);   
	CloudEntity.LazerSwitchCmd(1);
	//启动射击。
//    CloudEntity.Feed_Free_Once_Set(feed_gap, (bsp_dbus_Data.CH_0 > 200)); //供弹指令
	CloudEntity.Feed2nd.Free_Once_Set(100,&bsp_dbus_Data.CH_0);
    // if (bsp_dbus_Data.CH_0 > 200)    //状态信息发送到VisionTx
    //     VisionTx.Shoot_mode = 1;
    // else
    //     VisionTx.Shoot_mode = 0;

//    VisionTx.Shoot_mode = CloudEntity.shoot_flag; //状态信息发送到VisionTx

//    CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_2 * 10000.0f / 660.0f;
//    SUPERIOR_CHASSIS_MOVE_CanTx();

    // SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
    //             (float)(bsp_dbus_Data.CH_2 * 10000.0f / 660.0f),
    //             0);

    
}
int16_t last_CH0;
float v_feed_spd = 4000;
uint8_t no_use_vision_shoot;
/**
 * @brief 视觉控制云台，手动拨弹
 */
void VisionFeed()
{
	
    CloudEntity.ShooterSwitchCmd(1);                                 //启动射击。
	CloudEntity.LazerSwitchCmd(1);
    // CloudEntity.Feed2nd.Free_Once_Set(100, &bsp_dbus_Data.CH_0 ); //供弹指令
	if(last_CH0<=200 && bsp_dbus_Data.CH_0 > 200)
	{
        CloudEntity.trig_cnt++;
		CloudEntity.Shoot(v_feed_spd, 1,100U, ShtOnce, bsp_dbus_Data.CH_0);
	}
	last_CH0 = bsp_dbus_Data.CH_0;
//    VisionTx.Shoot_mode = CloudEntity.shoot_flag;                    //状态信息发送到VisionTx

    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        CloudEntity.SetAngleTo(CloudEntity.RealPitch + VisionRx.Pitch,
                               CloudEntity.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        CloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
        break;
    case speed_cloud:
        CloudEntity.PitchMotor.Angle_Set(CloudEntity.RealPitch + VisionRx.Pitch);
        CloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
        break;
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = hold_cloud; //处理完成标志。因为一个命令只会处理一次，处理后 置0

//    CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_2 * 10000.0f / 660.0f;
//    SUPERIOR_CHASSIS_MOVE_CanTx();
}
/**
  * @brief  全局安全模式
  */
void GlobalSafe() 
{
    CloudEntity.ShooterSwitchCmd(0);   
	CloudEntity.LazerSwitchCmd(0);
    CloudEntity.Safe_Set();                         //所有电机安全模式
    uint8_t data[8] = {0};                          //发送空字节。之后会改掉的
    SentryCanSend(&hcan2, SUPERIOR_SAFE, &data[0]); //通过CAN向其他的MCU发送安全模式指令
}
uint32_t hard_calib_trig_cnt;
/**
 * @brief 硬件校准模式
 * 
 */
void HardCalibration()
{
    taskENTER_CRITICAL();
    app_imu_Init();
    taskEXIT_CRITICAL();
    hard_calib_trig_cnt ++;
    CurrentMode = &ModeGlobalSafe;
}

