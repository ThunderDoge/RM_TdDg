#include "SentryChassisLogic.hpp"
//#define DEBUG

GlobalModeName GlobalMode;  
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;
GlobalModeName RecvCMD;
GlobalModeName GetGlobalMode(){
    return GlobalMode;
}

void ModeSelect(void)
{
#ifndef DEBUG
	GlobalMode = RecvCMD;
    switch (GlobalMode)
    {
    case MODE_VIISON_SHOOTING_TEST:
        VisionControl();
        break;
    default:
        Self.Safe_Set();
        break;
    }

#else 
Self.Fric.Speed_Set(-3500);
Self.FeedUp.Freefire_Set(3000);
//Self.FeedUp.PR_Handle();
#endif
}

void VisionControl() //ÊÓ¾õµ÷ÊÔ
{
	switch (CanRecv.SuperiorControlFlags)
	{
	case _SUPERIOR_CHASSIS_SPEED_SET_:
		Self.pidDriveLocation.PIDMax = SpeedMax;	//»Ö¸´
		Self.MotorSpeed_Set(CanRecv.ChassisSpeed);
		break;
	case _SUPERIOR_CHASSIS_LOACATION_SET_:
		Self.pidDriveLocation.PIDMax = SpeedMax;	//»Ö¸´
		Self.MotorSoftLocation_Set(Self.MotorSoftLocation + CanRecv.ChassisLocation);	
		break;
	case _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_:
		Self.MotorSoftLocation_Set(Self.MotorSoftLocation + CanRecv.ChassisLocation);	//¸²Ð´
		Self.pidDriveLocation.PIDMax = CanRecv.ChassisSpeedLimit;
		break;
	default:
		break;
	}
	CanRecv.SuperiorControlFlags = _SUPERIOR_OFFLINE_;
	if(HAL_GetTick() - CanRecv.RecvUpdateTime > 1000 )
	{
		Self.Safe_Set();
		GlobalMode = MODE_SAFE;
	}
}

