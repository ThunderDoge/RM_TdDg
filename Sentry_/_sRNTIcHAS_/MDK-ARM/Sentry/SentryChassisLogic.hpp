#ifndef __SENTRY_CHASSIS_LOGIC_H_
#define __SENTRY_CHASSIS_LOGIC_H_
#include "SentryChassis.hpp"
#include "SentryCanCommu.hpp"
#include "bsp_adc_deal.h"

enum GlobalModeName
{
    MODE_SAFE = 1,
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
//typedef void(*vivoFuncPtr)(void);
//typedef void(*CanCommuFuncPtr)(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);
//class GlobalModeAgent
//{
//private:
//public:
//    GlobalModeAgent(uint32_t mode_name,
//                    uint8_t vision_funcword,
//                    uint32_t can_commu_id)
//        : ModeName(mode_name),
//          VisionFuncWord(vision_funcword),
//          CanCommuID(can_commu_id){};
//    GlobalModeAgent(uint32_t mode_name,
//                    uint8_t vision_funcword,
//                    uint32_t can_commu_id,
//                    vivoFuncPtr handle,
//                    CanCommuFuncPtr canrecv,
//                    CanCommuFuncPtr cansend,
//                    vivoFuncPtr VisionRecv,
//                    vivoFuncPtr visionsend)
//        : ModeName(mode_name),
//          VisionFuncWord(vision_funcword),
//          CanCommuID(can_commu_id),
//          VisionRecv(VisionRecv),
//          CanRecv(canrecv),
//          VisionSend(visionsend),
//          CanSend(cansend){};

//    GlobalModeAgent(uint32_t mode_name,
//                    uint8_t vision_funcword,
//                    uint32_t can_commu_id,
//                    vivoFuncPtr handle,
//                    vivoFuncPtr active,
//                    vivoFuncPtr deactive,
//                    CanCommuFuncPtr canrecv,
//                    vivoFuncPtr VisionRecv,
//                    vivoFuncPtr visionsend,
//                    CanCommuFuncPtr cansend) : ModeName(mode_name),
//                                           VisionFuncWord(vision_funcword),
//                                           CanCommuID(can_commu_id),
//                                           Handle(handle),
//                                           Activate(active),
//                                           Deactivate(deactive),
//                                           VisionRecv(VisionRecv),
//                                           CanRecv(canrecv),
//                                           VisionSend(visionsend),
//                                           CanSend(cansend){};

//    uint32_t ModeName;
//    uint8_t VisionFuncWord;
//    uint32_t CanCommuID;
//    vivoFuncPtr Handle = NULL;
//    vivoFuncPtr Activate = NULL;
//    vivoFuncPtr Deactivate = NULL;
//    vivoFuncPtr VisionRecv = NULL;
//    CanCommuFuncPtr CanRecv = NULL;
//    vivoFuncPtr VisionSend = NULL;
//    CanCommuFuncPtr CanSend = NULL;
//};

GlobalModeName GetGlobalMode();
extern CommandSourceName CommandSource;
extern GlobalModeName RecvCMD;

void ModeSelect(void);
void SuperiorControl(); //ÊÓ¾õµ÷ÊÔ

#endif // __SENTRY_CHASSIS_LOGIC_H_
