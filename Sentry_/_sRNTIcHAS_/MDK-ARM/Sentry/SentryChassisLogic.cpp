#include "SentryChassisLogic.hpp"

GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
//CommandSourceName CommandSource;
GlobalModeName RecvCMD;
GlobalModeName GetGlobalMode()
{
    return GlobalMode;
}

//ģʽ����
app_Mode ModeSuperSuperiorControl(nullptr,SuperiorControl,nullptr);
//app_Mode ModeAutonomousDrive()
app_Mode ModeGlobalSafe(nullptr,GlobalSafe,nullptr);
//ģʽָ��
app_Mode *CurrentMode=&ModeGlobalSafe;
app_Mode *LastMode=&ModeGlobalSafe;

void ModeSelect(void)
{
	if((CanRx.SuperCon_ChassisMode != _chassis_save)/* && (HAL_GetTick()-CanRx.RecvUpdateTime) <1000 */)
		GlobalMode = MODE_VIISON_SHOOTING_TEST;
	
    switch (GlobalMode)
    {
    case MODE_VIISON_SHOOTING_TEST:
        SuperiorControl();
        break;
    default:
        ChassisEntity.Safe_Set();
        break;
    }
}

void ChassisCanRxHandle(void);	//������Ҫ���õĺ���
void SuperiorControl() //�Ӿ�����
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


