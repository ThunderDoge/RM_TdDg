/**
  * @file   SentryCloudLogic.cpp
  * @brief    �ڱ������߼�
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryDownCloudLogic.hpp"

//ģʽ��־��������
// GlobalModeName GlobalMode;
// GlobalModeName LastGlobalMode;
// CommandSourceName CommandSource;

//ģʽ����
app_Mode ModeManualChassis(NULL, ManualChassis, NULL);
app_Mode ModeManualShoot(ManualShootEnter, ManualShoot, nullptr);
app_Mode ModeManualFeed(nullptr, ManualFeed, nullptr);
app_Mode ModeManualShootGyro(ManualShoot_Gyro_Enter, ManualShoot_Gyro, nullptr);
app_Mode ModeVisionControl(VisionControlEnter, VisionControl, VisionControlExit);
app_Mode ModeAutoMove(nullptr, nullptr, nullptr);
app_Mode ModeGlobalSafe(nullptr, GlobalSafe, nullptr);
app_Mode ModeVisionFeed(nullptr, VisionFeed, nullptr);

Mode *LastMode = &ModeGlobalSafe;
Mode *CurrentMode = &ModeGlobalSafe;

extern sentry_vision_data VisionRx, VisionTx;
/**
  * @brief  ģʽѡ�����������߼�Դ�ڴ�
  */
void ModeSelect(void)
{
    int mode = bsp_dbus_Data.S1 * 10 + bsp_dbus_Data.S2;
    // LastGlobalMode = GlobalMode;
    LastMode = CurrentMode;

    switch (mode)
    {
    case 32: //��-�£��ֶ��ص���
        CurrentMode = &ModeManualChassis;
        break;
    case 12: //��-�£��ֶ�����̨
        CurrentMode = &ModeManualShoot;
        break;
    case 33: //˫�У��Ӿ�������̨ת��
        CurrentMode = &ModeVisionControl;
        break;
    case 31: //���ϣ��Ӿ�������̨���ֶ������������ҡ���Ҳ�Ϊ�����
        // CurrentMode = MODE_VIISON_SHOOTING_TEST;
        // VisionControl();
        // ManualFeed();
        CurrentMode = &ModeVisionFeed;
        break;
    case 13: //��-�У�ң����������̨ ������ģʽ��δ��ɡ�
        // CurrentMode = MODE_MANUAL_SHOOTING_TEST;
        // ManualShoot_Gyro();
        CurrentMode = &ModeManualShootGyro;
        break;
    case 11: //��-�ϣ��ֶ�����̨���ֶ������������ҡ���Ҳ�Ϊ�����
        // CurrentMode = MODE_FRIC_TEST;
        // ManualFeed();
        CurrentMode = &ModeManualFeed;
        break;
    case 22: //˫��
    case 23:
    case 21: //����
    default: //Ĭ��
        // CurrentMode = MODE_SAFE;
        // GlobalSafe();
        CurrentMode = &ModeGlobalSafe;
        break;
    }
    if (LastMode != CurrentMode)
    {
        LastMode->Exit();
        CurrentMode->Enter();
    }
    CurrentMode->Run();
}


/**
  * @brief  �Ӿ�������̨
  */
void VisionControl(void)
{
    //��ָ̨���

    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        CloudEntity.SetAngleTo(CloudEntity.RealPitch + VisionRx.Pitch,
                               CloudEntity.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        CloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
        break;
    case speed_cloud:
        CloudEntity.PitchMotor.Angle_Set(CloudEntity.RealPitch + VisionRx.Pitch);
        CloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
        break;
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = 0; //������ɱ�־����Ϊһ������ֻ�ᴦ��һ�Σ��������0


    // 		float pitch;
    // 		float yaw;
    // switch (VisionRx.Function_word)
    // {
    // case CMD_GIMBAL_RELATIVE_CONTROL:
    //     pitch = CloudEntity.RealPitch + VisionRx.Pitch;
    //     yaw = CloudEntity.RealYaw + VisionRx.Yaw;
    //     CloudEntity.SetAngleTo(pitch, yaw);
    //     break;
    // case CMD_GIMBAL_ABSOLUTE_CONTROL:
    //     CloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
    //     break;
    // case CMD_CHASSIS_CONTROL:
    //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE, VisionRx.Vx, 0.0f);
    //     break;
    // case CMD_CHASSIS_LOACTION_CONTROL:
    //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION, VisionRx.Px, 0.0f);
    //     break;
    // case CMD_CHASSIS_LOCATION_LIMIT_SPEED:
    //     SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED, VisionRx.Px, VisionRx.SpeedLimit);
    // default:
    //     break;
    // }
    // GlobalMode = MODE_VIISON_SHOOTING_TEST;
    // VisionRxHandle();
}
void VisionControlEnter(){
    IS_SUPERIOR_VISION_CTRL =1;
}
void VisionControlExit(){
    IS_SUPERIOR_VISION_CTRL =0;
}
/**
  * @brief  ң����������̨
  */
const float dbus_rate = -0.00005;
void ManualShootEnter()
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //���� Ŀ��Ƕ�Ϊ��ǰ�Ƕȡ����Է�ֹģʽ�л�ʱ�Ƕ�ͻ�䡣
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //��Ϊ���Խǿ���
}
void ManualShoot()
{
    //����֮ǰӦ���ѵ���ManualShootEnter()����
    //Ŀ��pitch��yaw��������dbus_rate*ҡ��ֵ����
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //�ฺܶ�š���Щ���ǵ������ġ�
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    CloudEntity.SetAngleTo(up_pitch, up_yaw);
}
/**
  * @brief  ң����������̨��������ģʽ
  */
void ManualShoot_Gyro()
{
    float up_pitch = CloudEntity.TargetPitch - bsp_dbus_Data.CH_1 * dbus_rate; //�ฺܶ�š���Щ���ǵ������ġ�
    float up_yaw = CloudEntity.TargetYaw + bsp_dbus_Data.CH_0 * dbus_rate;
    CloudEntity.SetAngleTo_Gyro(up_pitch, up_yaw);
}
void ManualShoot_Gyro_Enter()
{
    CloudEntity.TargetPitch = CloudEntity.RotatedImuAngle[1]; //���� Ŀ��Ƕ�Ϊ��ǰ�Ƕȡ����Է�ֹģʽ�л�ʱ�Ƕ�ͻ�䡣
    CloudEntity.TargetYaw = CloudEntity.RotatedImuAngle[2];
    CloudEntity.Mode = absolute_gyro_cloud; //��Ϊ���Խǿ���
}
/**
  * @brief  ң�������Ե���
  */
void ManualChassis() //�ֶ�����
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                  (float)(bsp_dbus_Data.CH_0 * 10000.0f / 660.0f),
                  0);
}
void ManualChassisEnter(){
    
}

/**
  * @brief  ң�������Բ���
  */
// float feed_speed;
void ManualFeed()
{
    // SentryCanSend(&CAN_INTERBOARD, UP_FEED, feed_speed, 0.0f);
    // CloudEntity.FricLeftMotor.Speed_Set(-Shoot_Speed);
    // CloudEntity.FricRightMotor.Speed_Set(Shoot_Speed);
    // CloudEntity.Feed2nd.Free_Once_Set(100, (bsp_dbus_Data.CH_0 > 200));

    CloudEntity.ShooterSwitchCmd(1);                                 //���������
    CloudEntity.Feed_Free_Once_Set(100, (bsp_dbus_Data.CH_0 > 200)); //����ָ��

    // if (bsp_dbus_Data.CH_0 > 200)    //״̬��Ϣ���͵�VisionTx
    //     VisionTx.Shoot_mode = 1;
    // else
    //     VisionTx.Shoot_mode = 0;

    VisionTx.Shoot_mode = CloudEntity.shoot_flag; //״̬��Ϣ���͵�VisionTx
    
}
/**
 * @brief �Ӿ�������̨���ֶ�����
 */
void VisionFeed()
{
    CloudEntity.ShooterSwitchCmd(1);                                 //���������
    CloudEntity.Feed_Free_Once_Set(100, (bsp_dbus_Data.CH_0 > 200)); //����ָ��
    VisionTx.Shoot_mode = CloudEntity.shoot_flag;                    //״̬��Ϣ���͵�VisionTx

    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        CloudEntity.SetAngleTo(CloudEntity.RealPitch + VisionRx.Pitch,
                               CloudEntity.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        CloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
        break;
    case speed_cloud:
        CloudEntity.PitchMotor.Angle_Set(CloudEntity.RealPitch + VisionRx.Pitch);
        CloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
        break;
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = 0; //������ɱ�־����Ϊһ������ֻ�ᴦ��һ�Σ������ ��0
}
/**
  * @brief  ȫ�ְ�ȫģʽ
  */
void GlobalSafe() //��ȫģʽ
{
    CloudEntity.Safe_Set();                         //���е����ȫģʽ
    uint8_t data[8] = {0};                          //���Ϳ��ֽڡ�֮���ĵ���
    SentryCanSend(&hcan2, SUPERIOR_SAFE, &data[0]); //ͨ��CAN��������MCU���Ͱ�ȫģʽָ��
}

