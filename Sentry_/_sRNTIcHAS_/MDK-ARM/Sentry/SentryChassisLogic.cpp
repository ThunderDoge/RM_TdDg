#include "SentryChassisLogic.hpp"
#define DEBUG

#ifdef DEBUG

pid pidPower(1, 0, 0, 1000, 10000, 10, 10);
float PowerOut;
float TargetPower = 1;
float FeedFricSpd = 3500;
float FeedUpSpd = 3000;
#endif // DEBUG

GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;
GlobalModeName RecvCMD;
GlobalModeName GetGlobalMode()
{
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
    {
        PowerOut += pidPower.pid_run(TargetPower - fabs(Self.DrivePower));
		if(PowerOut<0) PowerOut = 0;
        Self.MotorSpeed_Set(PowerOut);
    }
	{
		Self.FeedUp.Freefire_Set(FeedUpSpd);
		Self.Fric.Speed_Set(-FeedFricSpd);
	}
#endif
}

void VisionControl() //ÊÓ¾õµ÷ÊÔ
{
    switch (CanRecv.SuperiorControlFlags)
    {
    case _SUPERIOR_CHASSIS_SPEED_SET_:
        Self.pidDriveLocation.PIDMax = SpeedMax; //»Ö¸´
        Self.MotorSpeed_Set(CanRecv.ChassisSpeed);
        break;
    case _SUPERIOR_CHASSIS_LOACATION_SET_:
        Self.pidDriveLocation.PIDMax = SpeedMax; //»Ö¸´
        Self.MotorSoftLocation_Set(Self.MotorSoftLocation + CanRecv.ChassisLocation);
        break;
    case _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_:
        Self.MotorSoftLocation_Set(Self.MotorSoftLocation + CanRecv.ChassisLocation); //¸²Ð´
        Self.pidDriveLocation.PIDMax = CanRecv.ChassisSpeedLimit;
        break;
    default:
        break;
    }
    CanRecv.SuperiorControlFlags = _SUPERIOR_OFFLINE_;
    if (HAL_GetTick() - CanRecv.RecvUpdateTime > 1000)
    {
        Self.Safe_Set();
        GlobalMode = MODE_SAFE;
    }
}
