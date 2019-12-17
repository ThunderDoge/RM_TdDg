/**
  * @file  SentryModeAgents.cp
  * @brief    哨兵通用 模式通信执行代理人
  * @details  模式代理人(ModeAgent)是将某个功能的 通信与执行聚合起来的类
  * @author   ThunderDoge
  * @date     2019/12/17
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "SentryChassis.hpp"
#include "SentryCanCommu.hpp"
#include "bsp_adc_deal.h"


 //-------------------------功能处理代理人，最初的接口类
class GlobalAgent_Type0_Vir
{
public:
    GlobalAgent_Type0();

    uint32_t ModeCode;
    virtual void Handle() = 0;
};
//-------------------------CAN与视觉串口VisionUart通讯代理人
class GlobalAgent_Can_ViUart : public GlobalAgent_Type0_Vir
{
    GlobalAgent_Can_ViUart(uint32_t mode_name,
                           uint32_t can_commu_id,
                           uint8_t vision_funcword)
        : ModeName(mode_name),
          VisionFuncWord(vision_funcword),
          CanCommuID(can_commu_id){};
    uint8_t VisionFuncWord;
    uint32_t CanCommuID;
    virtual void CanSend(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data) = 0;
    virtual void CanRecv(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data) = 0;
    virtual void VisionSend(void) = 0;
    virtual void VisionRecv(void) = 0;
};

//!----------------来吧，甜蜜的死亡。哨兵云台状态信息通讯代理人
class CloudStateHandler : public GlobalAgent_Can_ViUart
{
public:
    CloudStateHandler(uint32_t mode_name,
                      uint32_t can_commu_id,
                      uint8_t vision_funcword)
        : GlobalAgent_Can_ViUart(mode_name,
                                 can_commu_id,
                                 vision_funcword){};
    //机上云台信息  Cloud State
    float RealAngle[2]; //![Pitch,Yaw] 下同
    float RealMechanical[2];
    //上级要求云台信息 Superior Command for Cloud
    float CommandAngle[2];
    
    void Handle();  //更新函数
};

void CloudStateHandler::Handle()
{
    RealAngle[1] = Self
}