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

///回调函数，直接执行接收到的小主机指令
void VisionRxHandle(void)
{
    //云台指令处理
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
    VisionRx.cloud_ctrl_mode = 0; //处理完成标志。因为一个命令只会处理一次，处理后置0
}
///与小主机通信任务用的回调函数
void CloudVisonTxRoutine(void)
{
    //装载各种信息
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

    //串口发送
    CMD_GET_MCU_STATE_Tx(VisionTx.Pitch, VisionTx.Yaw, VisionTx.YawSoft, VisionTx.Cloud_mode, VisionTx.Shoot_mode);
    ROBOT_ERR_Tx(VisionTx.Error_code);
    STA_CHASSIS_Tx(VisionTx.chassis_mode, VisionTx.pillar_flag, VisionTx.Vx, VisionTx.Px);
}
