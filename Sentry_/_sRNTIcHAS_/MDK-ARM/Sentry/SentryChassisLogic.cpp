#include "SentryChassisLogic.hpp"

GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;

void ModeSelect(void)
{
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
