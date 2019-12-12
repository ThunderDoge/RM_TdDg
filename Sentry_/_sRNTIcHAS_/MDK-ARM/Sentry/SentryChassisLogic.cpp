#include "SentryChassisLogic.hpp"

GlobalModeName GlobalMode;  
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;
GlobalModeName RecvCMD;
GlobalModeName GetGlobalMode(){
    return GlobalMode;
}

void ModeSelect(void)
{
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
}

void VisionControl() //ÊÓ¾õµ÷ÊÔ
{
	if(CanRecv.Ready_Flag)
	{
		switch (CanRecv.SuperiorControlFlags)
		{
		case _SUPERIOR_CHASSIS_SPEED_SET_:
			Self.MotorSpeed_Set(CanRecv.ChassisSpeed);
			break;
		case _SUPERIOR_CHASSIS_LOACATION_SET_:
			Self.MotorSoftLocation_Set(CanRecv.ChassisLocation);
			break;
		default:
			break;
		}
	}
	if(HAL_GetTick() - CanRecv.RecvUpdateTime > 1000 )
	{
		Self.Safe_Set();
		GlobalMode = MODE_SAFE;
	}
}
