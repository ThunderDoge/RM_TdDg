/**
  * @brief    �ڱ������߼�
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudLogic.hpp"

GlobalModeName GlobalMode;
GlobalModeName LastGlobalMode;
CommandSourceName CommandSource;
/**
  * @brief  �Ӿ�������̨
  */
void VisionControl(void)
{
	if(bsp_vision_Rec_Data.Ready_flag)
	{
		float pitch;
		float yaw;
		switch (bsp_vision_Rec_Data.Function_word)
		{
			case CMD_GIMBAL_RELATIVE_CONTROL:
				pitch = Self.RealPitch + bsp_vision_Rec_Data.Pitch;
				yaw = Self.RealYaw + bsp_vision_Rec_Data.Yaw;
				Self.SetAngleTo(pitch, yaw);
			break;
			case CMD_GIMBAL_ABSOLUTE_CONTROL:
				Self.SetAngleTo(bsp_vision_Rec_Data.Pitch , bsp_vision_Rec_Data.Yaw);
			break;
			default:
			break;
		}
		bsp_vision_Rec_Data.Ready_flag = 0;
	}
}
/**
  * @brief  ң����������̨
  */
const float dbus_rate = -0.00005;
void ManualShoot()
{
	if(GlobalMode != LastGlobalMode)
	{
		Self.TargetPitch = Self.RealPitch;
		Self.TargetYaw = Self.RealYaw;
	}
    float up_pitch = Self.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate;		//�ฺܶ�š���Щ���ǵ������ġ�
    float up_yaw = Self.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    Self.SetAngleTo(up_pitch, up_yaw);
//	float down_pitch = Self.RealPitch + bsp_dbus_Data.CH_3 * dbus_rate;
//    float down_yaw = Self.RealYaw + bsp_dbus_Data.CH_2 * dbus_rate;
//	Self.SetAngleTo(down_pitch,down_yaw);
}
/**
  * @brief  ң�������Ե���
  */
void ManualChassis() //�ֶ�����
{
    CloudCanSend(&hcan1, SUPERIOR_CHASSIS_MOVE,
                 (float)(bsp_dbus_Data.CH_0 * 10000.0f / 660.0f),
                 0);
}
/**
  * @brief  ȫ�ְ�ȫģʽ
  */
void GlobalSafe() //��ȫģʽ
{
    Self.Safe_Set();
    uint8_t data[8] = {0};
    CloudCanSend(&hcan2, SUPERIOR_SAFE, &data[0]);
    // CloudCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,uint8_t*  ptrData);
    // CloudCanSend(CAN_HandleTypeDef* _hcan,SENTRY_CAN_ID command_id,float argu1,float argu2);
}
/**
  * @brief  ģʽѡ�����������߼�Դ�ڴ�
  */
void ModeSelect(void)
{
    int mode = bsp_dbus_Data.S1 * 10 + bsp_dbus_Data.S2;
	LastGlobalMode = GlobalMode;
    switch (mode)
    {
    case 32: //��-��
        GlobalMode = MODE_MANUAL_CHASSIS_MOVE;
        ManualChassis();
        break;

    case 12: //��-��
        GlobalMode = MODE_MANUAL_SHOOTING_TEST;
        ManualShoot();
        break;
    case 33: //˫��
        GlobalMode = MODE_VIISON_SHOOTING_TEST;
        VisionControl();
        break;
    // case 13:    //��-��
    //     GlobalMode = MODE_AUTONOMOUS;
    //     AutoMove();
    //     break;
    case 22: //˫��
    case 23:
    case 21: //����
    default: //Ĭ��
        GlobalMode = MODE_SAFE;
        GlobalSafe();
        break;
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
