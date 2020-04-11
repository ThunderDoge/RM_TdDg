/**
  * @file   SentryDownCloudLogic.hpp
  * @brief    哨兵控制逻辑
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CLOUD_LOGIC_HPP
#define __SENTRY_CLOUD_LOGIC_HPP

#ifndef __PROJECT_SENTRY_DOWN_CLOUD_     //定义工程标识符__PROJECT_SENTRY_DOWN_CLOUD_
#define __PROJECT_SENTRY_DOWN_CLOUD_
#endif // __PROJECT_SENTRY_DOWN_CLOUD_

#include "bsp_dbus.h"
#include "SentryDownCloud.hpp"
#include "SentryDownCloudCommu.hpp"
#include "app_mode.hpp"

enum GlobalModeName : uint8_t
{
    MODE_SAFE = 0X00,
    MODE_MANUAL_SHOOTING_TEST,
    MODE_VIISON_SHOOTING_TEST,
    MODE_FRIC_TEST,
    MODE_KEYBOARD_TEST,
    MODE_MANUAL_CHASSIS_MOVE,
    MODE_AUTONOMOUS,
};

// extern GlobalModeName GlobalMode;
// extern CommandSourceName CommandSource;
extern SentryCloud CloudEntity;

extern app_Mode* CurrentMode,*LastMode;

/**
 * @defgroup RemoteDebugModes
 * @addtogroup RemoteDebugModes
 * @{
 */
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

/** @} */
#endif // __SENTRY_CLOUD_LOGIC_HPP

/**
  * @brief  废案 Abandonded Code
  */
// enum CommandSourceName
// {
//     CMDSRC_DBUS,
//     CMDSRC_CAN,
//     CMDSRC_SELF,
// };

// class GlobalModeClass
// {
// public:

//     uint32_t ModeName;
//     uint8_t VisionFuncWord;
//     uint32_t CanCommuID;
//     void Handle();
//     void Activate();
//     void Deactivate();
//     void VisionRecv();
//     void CanRecv();
//     void VisionSend();
//     void CanSend();
// }

// class GlobalModeClass
// {
// private:
// public:
//     GlobalModeClass(uint32_t mode_name,
//                     uint8_t vision_funcword,
//                     uint32_t can_commu_id)
//         : ModeName(mode_name),
//           VisionFuncWord(vision_funcword),
//           CanCommuID(can_commu_id){};
//     GlobalModeClass(uint32_t mode_name,
//                     uint8_t vision_funcword,
//                     uint32_t can_commu_id,
//                     vivoFuncPtr handle,
//                     vivoFuncPtr active,
//                     vivoFuncPtr deactive,
//                     vivoFuncPtr canrecv,
//                     vivoFuncPtr VisionRecv,
//                     vivoFuncPtr visionsend,
//                     vivoFuncPtr cansend) : ModeName(mode_name),
//                                            VisionFuncWord(vision_funcword),
//                                            CanCommuID(can_commu_id),
//                                            Handle(handle),
//                                            Activate(active),
//                                            Deactivate(deactive),
//                                            VisionRecv(VisionRecv),
//                                            CanRecv(canrecv),
//                                            VisionSend(visionsend),
//                                            CanSend(cansend){};

//     uint32_t ModeName;
//     uint8_t VisionFuncWord;
//     uint32_t CanCommuID;

//     // void (*Handle)(void) = NULL;
//     // void (*Activate)(void) = NULL;
//     // void (*Deactivate)(void) = NULL;
//     // void (*VisionRecv)(void) = NULL;
//     // void (*CanRecv)(void) = NULL;
//     // void (*VisionSend)(void) = NULL;
//     // void (*CanSend)(void) = NULL;
//     vivoFuncPtr Handle = NULL;
//     vivoFuncPtr Activate = NULL;
//     vivoFuncPtr Deactivate = NULL;
//     vivoFuncPtr VisionRecv = NULL;
//     vivoFuncPtr CanRecv = NULL;
//     vivoFuncPtr VisionSend = NULL;
//     vivoFuncPtr CanSend = NULL;
// };
