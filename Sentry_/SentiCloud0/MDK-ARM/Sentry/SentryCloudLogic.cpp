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

//模式标志变量定义
// GlobalModeName GlobalMode;
// GlobalModeName LastGlobalMode;
// CommandSourceName CommandSource;

//模式定义
app_Mode ModeManualChassis(NULL, ManualChassis, NULL);
app_Mode ModeManualShoot(ManualShootEnter, ManualShoot, nullptr);
app_Mode ModeManualFeed(nullptr, ManualFeed, nullptr);
app_Mode ModeManualShootGyro(ManualShoot_Gyro_Enter, ManualShoot_Gyro, nullptr);
app_Mode ModeVisionControl(VisionControlEnter, VisionControl, VisionControlExit);
app_Mode ModeAutoMove(nullptr, nullptr, nullptr);
app_Mode ModeGlobalSafe(nullptr, GlobalSafe, nullptr);
app_Mode ModeVisionFeed(nullptr, VisionFeed, nullptr);

app_Mode *LastMode = &ModeGlobalSafe;
app_Mode *CurrentMode = &ModeGlobalSafe;

extern sentry_vision_data VisionRx, VisionTx;
int t=4;
float p=0,y=0;
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
    case 31: //中上：视觉控制云台，手动供弹射击（右摇杆右拨为扳机）
        // CurrentMode = MODE_VIISON_SHOOTING_TEST;
        // VisionControl();
        // ManualFeed();
        CurrentMode = &ModeVisionFeed;
        break;
    case 11: //上-上：遥控器测试云台 陀螺仪模式【未完成】
        // CurrentMode = MODE_MANUAL_SHOOTING_TEST;
        // ManualShoot_Gyro();
        CurrentMode = &ModeManualShootGyro;
        break;
    case 13: //上-中：手动控云台，手动供弹射击（右摇杆右拨为扳机）
        // CurrentMode = MODE_FRIC_TEST;
        // ManualFeed();
        CurrentMode = &ModeManualFeed;
        break;
    case 22: //双下
    case 23:
    case 21: //左下
    default: //默认
        // CurrentMode = MODE_SAFE;
        // GlobalSafe();
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
    VisionRx.cloud_ctrl_mode = hold_cloud; //处理完成标志。因为一个命令只会处理一次，处理后置0
	CloudEntity.LazerSwitchCmd(1);

}
void VisionControlEnter(){
    IS_SUPERIOR_VISION_CTRL =1;
}
void VisionControlExit(){
    IS_SUPERIOR_VISION_CTRL =0;
}
/**
  * @brief  遥控器测试云台
  */
const float dbus_rate = -0.00005;
void ManualShootEnter()
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //视为绝对角控制
}
void ManualShoot()
{
    //在这之前应当已调用ManualShootEnter()函数
    //目标pitch和yaw受灵敏度dbus_rate*摇杆值控制
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    CloudEntity.SetAngleTo(up_pitch, up_yaw);
	CloudEntity.LazerSwitchCmd(1);
	CloudEntity.ShooterSwitchCmd(0);   
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                  (float)(bsp_dbus_Data.CH_2 * 10000.0f / 660.0f),
                  0);
}
/**
  * @brief  遥控器测试云台，陀螺仪模式
  */
void ManualShoot_Gyro()
{
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    CloudEntity.SetAngleTo_Gyro(up_pitch, up_yaw);
}
void ManualShoot_Gyro_Enter()
{
    CloudEntity.TargetPitch = CloudEntity.RotatedImuAngle[1]; //重置 目标角度为当前角度。用以防止模式切换时角度突变。
    CloudEntity.TargetYaw = CloudEntity.RotatedImuAngle[2];
    CloudEntity.Mode = absolute_gyro_cloud; //视为绝对角控制
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

    VisionTx.Shoot_mode = CloudEntity.shoot_flag; //状态信息发送到VisionTx

    CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_2 * 10000.0f / 660.0f;
    SUPERIOR_CHASSIS_MOVE_CanTx();

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

	last_CH0 = bsp_dbus_Data.CH_0;
    VisionTx.Shoot_mode = CloudEntity.shoot_flag;                    //状态信息发送到VisionTx

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

	
    CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_2 * 10000.0f / 660.0f;
    SUPERIOR_CHASSIS_MOVE_CanTx();
}
/**
  * @brief  全局安全模式
  */
void GlobalSafe() //安全模式
{
    CloudEntity.ShooterSwitchCmd(0);   
	CloudEntity.LazerSwitchCmd(0);
    CloudEntity.Safe_Set();                         //所有电机安全模式
    uint8_t data[8] = {0};                          //发送空字节。之后会改掉的
    SentryCanSend(&hcan2, SUPERIOR_SAFE, &data[0]); //通过CAN向其他的MCU发送安全模式指令
}

