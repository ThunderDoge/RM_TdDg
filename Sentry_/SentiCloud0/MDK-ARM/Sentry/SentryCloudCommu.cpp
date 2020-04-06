/**
 * @file SentryCloudCommu.cpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "SentryCloudCommu.hpp"

///�ص�������ֱ��ִ�н��յ���С����ָ��
void VisionRxHandle(void)
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
}
///��С����ͨ�������õĻص�����
void CloudVisonTxRoutine(void)
{
    //װ�ظ�����Ϣ
    VisionTx.Cloud_mode = CloudEntity.Mode;
    VisionTx.Shoot_mode = CloudEntity.shoot_flag;
    VisionTx.Pitch = CloudEntity.RealPitch;
    VisionTx.YawSoft = CloudEntity.RealYaw;
    VisionTx.Yaw = CloudEntity.MechanicYaw;
    VisionTx.Shoot_speed = 0;

    VisionTx.Error_code = 0;

    VisionTx.chassis_mode = VisionRx.chassis_mode;
    VisionTx.Vx = CanRx.Chassis_SpeedLocation[0];
    VisionTx.pillar_flag = CanRx.Pillar_flag;
    VisionTx.Px = CanRx.Chassis_SpeedLocation[1];
    VisionTx.pillar_flag = CanRx.Pillar_flag;

    //���ڷ���
    CMD_GET_MCU_STATE_Tx();
    ROBOT_ERR_Tx();
    STA_CHASSIS_Tx();
}
