/**
  * @brief    哨兵控制逻辑
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudLogic.hpp"
#include "SentryCommu.hpp"
GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;

extern Sentry_vision_data VisionRx,VisionTx;

//GlobalModeClass GlobalSafeMode((uint32_t)MODE_SAFE, (uint8_t)0, (uint32_t)SUPERIOR_SAFE);

    /**
  * @brief  视觉控制云台
  */
    void VisionControl(void)
{
        // 		float pitch;
        // 		float yaw;
        // switch (VisionRx.Function_word)
        // {
        // case CMD_GIMBAL_RELATIVE_CONTROL:
        //     pitch = Self.RealPitch + VisionRx.Pitch;
        //     yaw = Self.RealYaw + VisionRx.Yaw;
        //     Self.SetAngleTo(pitch, yaw);
        //     break;
        // case CMD_GIMBAL_ABSOLUTE_CONTROL:
        //     Self.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
        //     break;
        // case CMD_CHASSIS_CONTROL:
        //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE, VisionRx.Vx, 0.0f);
        //     break;
        // case CMD_CHASSIS_LOACTION_CONTROL:
        //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION, VisionRx.Px, 0.0f);
        //     break;
        // case CMD_CHASSIS_LOCATION_LIMIT_SPEED:
        //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED, VisionRx.Px, VisionRx.SpeedLimit);
        // default:
        //     break;
        // }
		GlobalMode = MODE_VIISON_SHOOTING_TEST;
        VisionRxHandle();
}
/**
  * @brief  遥控器测试云台
  */
const float dbus_rate = -0.00005;
void ManualShoot()
{
    if (GlobalMode != LastGlobalMode)
    {
        Self.TargetPitch = Self.RealPitch;
        Self.TargetYaw = Self.RealYaw;
    }
    float up_pitch = Self.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = Self.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    Self.SetAngleTo(up_pitch, up_yaw);
    //	float down_pitch = Self.RealPitch + bsp_dbus_Data.CH_3 * dbus_rate;
    //    float down_yaw = Self.RealYaw + bsp_dbus_Data.CH_2 * dbus_rate;
    //	Self.SetAngleTo(down_pitch,down_yaw);
}
/**
  * @brief  遥控器测试云台，陀螺仪模式
  */
void ManualShoot_Gyro()
{
    if (GlobalMode != LastGlobalMode)
    {
        Self.TargetPitch = Self.RotatedImuAngle[1];
        Self.TargetYaw = Self.RotatedImuAngle[2];
    }
    float up_pitch = Self.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //很多负号。这些都是调出来的。
    float up_yaw = Self.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    Self.SetAngleTo_Gyro(up_pitch, up_yaw);
}
/**
  * @brief  遥控器测试底盘
  */
void ManualChassis() //手动底盘
{
	GlobalMode = MODE_MANUAL_CHASSIS_MOVE;
}
/**
  * @brief  遥控器测试拨弹
  */
float feed_speed;
int32_t Shoot_Speed=7000;
void ManualFeed()
{
	Self.shoot_flag = 1;
	SentryCanSend(&CAN_INTERBOARD,UP_FEED,feed_speed,0.0f);
	Self.FricLeftMotor.Speed_Set(-Shoot_Speed);
	Self.FricRightMotor.Speed_Set(Shoot_Speed);
	Self.Feed2nd.Free_Once_Set(100,(bsp_dbus_Data.CH_0>200));
	
	if(bsp_dbus_Data.CH_0>200) VisionTx.Shoot_mode = 1;
	else VisionTx.Shoot_mode = 0;
}

/**
  * @brief  全局安全模式
  */
void GlobalSafe() //安全模式
{
    Self.Safe_Set();
    uint8_t data[8] = {0};
    SentryCanSend(&hcan2, SUPERIOR_SAFE, &data[0]);
    // SentryCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,uint8_t*  ptrData);
    // SentryCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,float argu1,float argu2);
}
/**
  * @brief  模式选择函数，控制逻辑源于此
  */
void ModeSelect(void)
{
    int mode = bsp_dbus_Data.S1 * 10 + bsp_dbus_Data.S2;
    LastGlobalMode = GlobalMode;
	
	Self.shoot_flag = 0;
	
    switch (mode)
    {
    case 32: //中-下
        GlobalMode = MODE_MANUAL_CHASSIS_MOVE;
        ManualChassis();
        break;

    case 12: //上-下
        GlobalMode = MODE_MANUAL_SHOOTING_TEST;
        ManualShoot();
        break;
    case 33: //双中
        GlobalMode = MODE_VIISON_SHOOTING_TEST;
        VisionControl();
        break;
	case 31:	//中上
         GlobalMode = MODE_VIISON_SHOOTING_TEST;
		VisionControl();
         ManualFeed();
         break;
     case 13:    //上-中
		GlobalMode = MODE_MANUAL_SHOOTING_TEST;
		ManualShoot_Gyro();
		break;
	case 11:	//上-上
		GlobalMode = MODE_FRIC_TEST;
		ManualFeed();

		break;
    case 22: //双下
    case 23:
    case 21: //左下
    default: //默认
        GlobalMode = MODE_SAFE;
        GlobalSafe();
        break;
    }
	if(Self.shoot_flag==0)
	{
		Self.Feed2nd.Safe_Set();
	}
}
/**
  * @brief  
  * @details  
  * @param[in]  
  * @retval  
  */
void CloudCommuRoutine(void)
{
}
