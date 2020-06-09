/**
  * @file   SentryCloudLogic.hpp
  * @brief    哨兵控制逻辑
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CLOUD_LOGIC_HPP
#define __SENTRY_CLOUD_LOGIC_HPP


#include "sentry_dwncld_config.h"    // 工程标识宏定义

#include "bsp_dbus.h"               // 遥控器支持包
#include "SentryDownCloud.hpp"          // 云台
#include "SentryDownCloudCommu.hpp"     // 云台通讯
#include "app_mode.hpp"             // class【模式】

extern SentryCloud CloudEntity;     // 云台实体

extern app_Mode* CurrentMode,*LastMode; // 模式指针，指示目前的模式

extern app_Mode ModeManualChassis, ModeManualChassis, ModeManualShoot, ModeVisionControl, ModeAutoMove, ModeGlobalSafe; ///模式对象列表

void ModeSelect(); ///主逻辑-模式选择

void ManualChassis(); ///手动底盘
void ManualShoot();   ///手动操炮射击
void ManualShootEnter();
void ManualShoot_Gyro();
void ManualShoot_Gyro_Enter();
void ManualFeed();
void VisionFeed();

void VisionControl(); ///视觉调试
void VisionControlEnter();
void VisionControlExit();
void AutoMove();      ///全自动移动
void GlobalSafe();    ///安全模式
void HardCalibration();	///运行时陀螺仪零漂校准

#endif // __SENTRY_CLOUD_LOGIC_HPP
