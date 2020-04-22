/**
  * @file   SentryCloudLogic.cpp
  * @brief    哨兵控制逻辑
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryDownCloudLogic.hpp"

//模式标志变量定义
// GlobalModeName GlobalMode;
// GlobalModeName LastGlobalMode;
// CommandSourceName CommandSource;

//模式定义
app_Mode ModeVisionControl(NULL, VisionControl, NULL);
app_Mode ModeGlobalSafe(NULL, GlobalSafe, NULL);

app_Mode *LastMode = &ModeGlobalSafe;
app_Mode *CurrentMode = &ModeGlobalSafe;

extern sentry_vision_data VisionRx, VisionTx;
/**
  * @brief  模式选择函数，控制逻辑源于此
  */
void ModeSelect(void)
{
    if (LastMode != CurrentMode)
    {
        LastMode->Exit();
        CurrentMode->Enter();
    }
    CurrentMode->Run();
}


/**
  * @brief  视觉控制云台
  */
void VisionControl(void)
{
    //云台指令处理

    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        DownCloudEntity.SetAngleTo(DownCloudEntity.RealPitch + VisionRx.Pitch,
                               DownCloudEntity.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        DownCloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
        break;
    case speed_cloud:
        DownCloudEntity.PitchMotor.Angle_Set(DownCloudEntity.RealPitch + VisionRx.Pitch);
        DownCloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
        break;
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = 0; //处理完成标志。因为一个命令只会处理一次，处理后置0
}

/**
  * @brief  全局安全模式
  */
void GlobalSafe() //安全模式
{
    DownCloudEntity.Safe_Set();                         //所有电机安全模式
    uint8_t data[8] = {0};                          //发送空字节。之后会改掉的
    SentryCanSend(&hcan2, SUPERIOR_SAFE, &data[0]); //通过CAN向其他的MCU发送安全模式指令
}

