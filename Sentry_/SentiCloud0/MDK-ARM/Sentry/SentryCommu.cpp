/**
  * @file  SentryCommu.hpp
  * @brief    哨兵视觉、CAN通讯标识符/函数汇总
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/18
  * @version  v1.0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "SentryCommu.hpp"

// Infomation Storage
CanCommuRecv_t CanInfo, CanRx, CanTx;
#ifdef USE_VISION
Sentry_vision_data VisionRx, VisionTx;
#endif



#ifdef CLOUD_COMMU
void CloudCanCommuRoutine(void)
{
	if(GlobalMode ==  MODE_VIISON_SHOOTING_TEST)
	{
    CanTx.SuperCon_ChassisMode = VisionRx.chassis_mode;
    CanTx.SuperCon_ChassisSpeedLocation[0] = VisionRx.Vx;
    CanTx.SuperCon_ChassisSpeedLocation[1] = VisionRx.Px;
    CanTx.SuperiorControlFlags = 1;
	}
	if(GlobalMode == MODE_MANUAL_CHASSIS_MOVE)
 {
		CanTx.SuperCon_ChassisMode = _chassis_speed;
		CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_0 * 8000.0f / 660.0f;
		CanTx.SuperiorControlFlags = 1;
	}
	SUPERIOR_CHASSIS_MOVE_CanTx();
    SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx(); //
}
#endif
#ifdef CHASSIS_COMMU
void ChassisCanCommuRoutine(void)
{
    CanTx.Chassis_SpeedLocation[0] = Self.MotorSpeed;
    CanTx.Chassis_SpeedLocation[1] = Self.MotorSoftLocation;
    CHASSIS_STATES_CanTx();
	CanTx.Pillar_flag = Self.PillarFlag;
	CHASSIS_PILLAR_CanTx();
}
//CAN信息底盘托管控制程序
void ChassisCanRxHandle(void)
{
	Self.Mode = (_chassis_mode) CanRx.SuperCon_ChassisMode;
	switch (CanRx.SuperCon_ChassisMode)
	{
	case _chassis_speed:
        Self.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location:
        Self.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed:
        Self.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
		CanRx.Chassis_SpeedLimit);
		break;
	}
}
#endif
