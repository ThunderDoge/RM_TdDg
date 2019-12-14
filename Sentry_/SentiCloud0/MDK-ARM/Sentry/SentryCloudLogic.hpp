/**
 * @brief    哨兵控制逻辑
 * @details  Encoding - GB2312
 * @author   OnePointFive 2020 ThunderDoge
 * @date     2019/12/13, The Day of
 * @version
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#ifndef __SENTRY_CLOUD_LOGIC_HPP
#define __SENTRY_CLOUD_LOGIC_HPP

#include "SentryCanCommu.hpp"
#include "SentryCloud.hpp"
#include "bsp_dbus.h"
#include "bsp_vision.hpp"

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
typedef void (*vivoFuncPtr)(void);
class GlobalModeClass
{
private:
public:
    GlobalModeClass(uint32_t mode_name,
                    uint8_t vision_funcword,
                    uint32_t can_commu_id)
        : ModeName(mode_name),
          VisionFuncWord(vision_funcword),
          CanCommuID(can_commu_id){};
    GlobalModeClass(uint32_t mode_name,
                    uint8_t vision_funcword,
                    uint32_t can_commu_id,
                    vivoFuncPtr handle,
                    vivoFuncPtr active,
                    vivoFuncPtr deactive,
                    vivoFuncPtr canrecv,
                    vivoFuncPtr VisionRecv,
                    vivoFuncPtr visionsend,
                    vivoFuncPtr cansend) : ModeName(mode_name),
                                           VisionFuncWord(vision_funcword),
                                           CanCommuID(can_commu_id),
                                           Handle(handle),
                                           Activate(active),
                                           Deactivate(deactive),
                                           VisionRecv(VisionRecv),
                                           CanRecv(canrecv),
                                           VisionSend(visionsend),
                                           CanSend(cansend){};

    uint32_t ModeName;
    uint8_t VisionFuncWord;
    uint32_t CanCommuID;

    // void (*Handle)(void) = NULL;
    // void (*Activate)(void) = NULL;
    // void (*Deactivate)(void) = NULL;
    // void (*VisionRecv)(void) = NULL;
    // void (*CanRecv)(void) = NULL;
    // void (*VisionSend)(void) = NULL;
    // void (*CanSend)(void) = NULL;
    vivoFuncPtr Handle = NULL;
    vivoFuncPtr Activate = NULL;
    vivoFuncPtr Deactivate = NULL;
    vivoFuncPtr VisionRecv = NULL;
    vivoFuncPtr CanRecv = NULL;
    vivoFuncPtr VisionSend = NULL;
    vivoFuncPtr CanSend = NULL;
};

enum GlobalModeName
{
    MODE_SAFE = 0X01,
    MODE_MANUAL_SHOOTING_TEST,
    MODE_VIISON_SHOOTING_TEST,
    MODE_KEYBOARD_TEST,
    MODE_MANUAL_CHASSIS_MOVE,
    MODE_AUTONOMOUS,
};
enum CommandSourceName
{
    CMDSRC_DBUS,
    CMDSRC_CAN,
    CMDSRC_SELF,
};

extern GlobalModeName GlobalMode;
extern CommandSourceName CommandSource;

void ManualChassis(); //手动底盘
void ManualShoot();   //手动操炮射击
void VisionControl(); //视觉调试
void AutoMove();      //全自动移动
void GlobalSafe();    //安全模式

void ModeSelect();

void CloudCommuRoutine(void);

#endif // __SENTRY_CLOUD_LOGIC_HPP
