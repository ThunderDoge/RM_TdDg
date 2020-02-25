#include "SentryChassisLogic.hpp"

uint8_t control_mode;
uint8_t pillar_close_flag;
float location_on_rail;
// void ChassisSendInfoHandle(void)
// {
//     control_mode = 0;
//     if (bsp_ADC1_Sharp_Distance[0] <= 13.0f)
//         pillar_close_flag = 1;
//     else if (bsp_ADC1_Sharp_Distance[1] <= 13.0f)
//         pillar_close_flag = 2;
//     else
//         pillar_close_flag = 0;
//     location_on_rail = ChassisEntity.MotorSoftLocation;
// }
// void ChassisSendInfoCanTx(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
// {
//     if(RxHead->StdId) == can_commu_id
// 	{
	
//     }
// }
//GlobalModeAgent ChassisSendInfo(0, 0X13, CHASSIS_STATES,
//ChassisSendInfoHandle,ChassisSendInfoCanTx );

//#define DEBUG

#ifdef DEBUG

pid pidDriveCurrent(1, 0, 0, 1000, 10000, 10, 10);
float PowerOut;
float TargetPower = 0;
float FeedFricSpd = 0;
float FeedUpSpd = 0;
float FeedUpRealSpd = 0;
float FeedUpRealCrr;
int16_t trig = 1;
int16_t dictT = 100;
#endif // DEBUG

GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
//CommandSourceName CommandSource;
GlobalModeName RecvCMD;
GlobalModeName GetGlobalMode()
{
    return GlobalMode;
}

//ģʽ����
Mode ModeSuperSuperiorControl(nullptr,SuperiorControl,nullptr);
Mode ModeGlobalSafe(nullptr,GlobalSafe,nullptr);
//ģʽָ��
Mode *CurrentMode=&ModeGlobalSafe;
Mode *LastMode=&ModeGlobalSafe;

void ModeSelect(void)
{
#ifndef DEBUG
//    GlobalMode = RecvCMD;
	if((CanRx.SuperCon_ChassisMode != _chassis_save) && (HAL_GetTick()-CanRx.RecvUpdateTime) <1000 )
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
#else
    {
        PowerOut += pidDriveCurrent.pid_run(TargetPower - fabs(ChassisEntity.DrivePower));
        if (PowerOut < 0)
            PowerOut = 0;
        ChassisEntity.MotorSpeed_Set(PowerOut);
    }
    {
        ChassisEntity.FeedUp.Freefire_Set(FeedUpSpd);
		if(trig >300)
			ChassisEntity.FeedUp.FreeOnce_Set(dictT,&trig);
        ChassisEntity.Fric.Speed_Set(-FeedFricSpd);
        FeedUpRealSpd = ChassisEntity.FeedUp.RealSpeed;
        FeedUpRealCrr = ChassisEntity.FeedUp.RealCurrent;
    }
#endif
}

void ChassisCanRxHandle(void);	//������Ҫ���õĺ���
void SuperiorControl() //�Ӿ�����
{
//    switch (CanRecv.SuperiorControlFlags)
//    {
//    case _SUPERIOR_CHASSIS_SPEED_SET_:
//        ChassisEntity.pidDriveLocation.PIDMax = SpeedMax; //�ָ�
//        ChassisEntity.MotorSpeed_Set(CanRecv.ChassisSpeed);
//        break;
//    case _SUPERIOR_CHASSIS_LOACATION_SET_:
//        ChassisEntity.pidDriveLocation.PIDMax = SpeedMax; //�ָ�
//        ChassisEntity.MotorSoftLocation_Set(ChassisEntity.MotorSoftLocation + CanRecv.ChassisLocation);
//        break;
//    case _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_:
//        ChassisEntity.MotorSoftLocation_Set(ChassisEntity.MotorSoftLocation + CanRecv.ChassisLocation); //��д
//        ChassisEntity.pidDriveLocation.PIDMax = CanRecv.ChassisSpeedLimit;
//        break;
//    default:
//        break;
//    }
//    CanRecv.SuperiorControlFlags = _SUPERIOR_OFFLINE_;
//    if (HAL_GetTick() - CanRecv.RecvUpdateTime > 1000)
//    {
//        ChassisEntity.Safe_Set();
//        GlobalMode = MODE_SAFE;
//    }
	ChassisCanRxHandle();
}
void GlobalSafe()
{
    ChassisEntity.Safe_Set();
}
#ifdef DEBUG
#endef
#endif


