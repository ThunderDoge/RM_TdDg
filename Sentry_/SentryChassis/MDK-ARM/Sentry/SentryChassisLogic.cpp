/**
 * @file      SentryChassisLogic.cpp
 * @brief     哨兵底盘运行逻辑
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-12
 * @version   1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 */
#include "SentryChassisLogic.hpp"

//模式定义
app_Mode ModeSuperSuperiorControl(NULL,SuperiorControl,NULL);
//app_Mode ModeAutonomousDrive()
app_Mode ModeGlobalSafe(NULL,GlobalSafe,NULL);
app_Mode ModeAutomonus(NULL,Automonus,NULL);
//模式指针
app_Mode *CurrentMode=&ModeGlobalSafe;
app_Mode *LastMode=&ModeGlobalSafe;

uint8_t force_mode=0;
/**
 * @brief 模式选择器。此为所有逻辑起始处
 * 
 */
void ModeSelect(void)
{
    LastMode = CurrentMode;
	switch(force_mode)
	{
		case 1:
			CurrentMode = &ModeSuperSuperiorControl;
			break;
		case 2:
			CurrentMode=&ModeAutomonus;
			break;
		case 0:
		default:
			if(GlobalCheckList.IsOfflineList[id_UpCloudConnect] && GlobalCheckList.IsOfflineList[id_DownCloudConnect])
			{
				CurrentMode=&ModeAutomonus;
			}
			else
			{
				CurrentMode=&ModeSuperSuperiorControl;
			}
			break;
	}

    if(LastMode!=CurrentMode)
    {
        LastMode->Exit();
        CurrentMode->Enter();
    }
    CurrentMode->Run();
}

void SuperiorControl() //视觉调试
{
	ChassisEntity.Mode = (_chassis_mode) CanRx.SuperCon_ChassisMode;
	switch (CanRx.SuperCon_ChassisMode)
	{
	case _chassis_speed:    // 底盘单速度环控制
        ChassisEntity.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location: // 底盘位置环控制
        ChassisEntity.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed: // 底盘位置环限速移动
        ChassisEntity.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
		CanRx.Chassis_SpeedLimit);
		break;
	default: 
		ChassisEntity.Safe_Set();
		break;
	}
}

void GlobalSafe()
{
    ChassisEntity.Safe_Set();
}



/**
 * @brief 自行运动模式
 * 
 */
void Automonus(void)
{
    if( !app_check_IsOffline(id_UpCloudConnect) || !app_check_IsOffline(id_DownCloudConnect)  )  // 检查有没有上线
    {
        CurrentMode=&ModeSuperSuperiorControl;
        return;
    }
	
//	if( GET_DMG_IN_LAST_5s)
//	{
//		ENABLE_USE_BUF_POW;
//	}
//	else
//	{
//		DISABLE_USE_BUF_POW;
//	}
//	
//	if( REACHED_LAST_LOCATION )
//	{
//		GERERATE_NEXT_LOCATION();
//	}
//	CTRL_TO_LOCATION();
	ChassisEntity.Safe_Set();
}




