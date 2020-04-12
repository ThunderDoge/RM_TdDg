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
app_Mode ModeSuperSuperiorControl(nullptr,SuperiorControl,nullptr);
//app_Mode ModeAutonomousDrive()
app_Mode ModeGlobalSafe(nullptr,GlobalSafe,nullptr);
//模式指针
app_Mode *CurrentMode=&ModeGlobalSafe;
app_Mode *LastMode=&ModeGlobalSafe;


/**
 * @brief 模式选择器。此为所有逻辑起始处
 * 
 */
void ModeSelect(void)
{
	// if((CanRx.SuperCon_ChassisMode != _chassis_save)/* && (HAL_GetTick()-CanRx.RecvUpdateTime) <1000 */)
	// 	GlobalMode = MODE_VIISON_SHOOTING_TEST;
	
    // switch (GlobalMode)
    // {
    // case MODE_VIISON_SHOOTING_TEST:
    //     SuperiorControl();
    //     break;
    // default:
    //     ChassisEntity.Safe_Set();
    //     break;
    // }
    if
}

void ChassisCanRxHandle(void);	//声明将要调用的函数
void SuperiorControl() //视觉调试
{
	ChassisEntity.Mode = (_chassis_mode) CanRx.SuperCon_ChassisMode;
	switch (CanRx.SuperCon_ChassisMode)
	{
	case _chassis_speed:
        ChassisEntity.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location:
        ChassisEntity.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed:
        ChassisEntity.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
		CanRx.Chassis_SpeedLimit);
		break;
	}
}
void GlobalSafe()
{
    ChassisEntity.Safe_Set();
}
#ifdef DEBUG
#endef
#endif


