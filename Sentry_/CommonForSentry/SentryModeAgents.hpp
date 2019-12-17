/**
  * @file  SentryModeAgents.cp
  * @brief    �ڱ�ͨ�� ģʽͨ��ִ�д�����
  * @details  ģʽ������(ModeAgent)�ǽ�ĳ�����ܵ� ͨ����ִ�оۺ���������
  * @author   ThunderDoge
  * @date     2019/12/17
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "SentryChassis.hpp"
#include "SentryCanCommu.hpp"
#include "bsp_adc_deal.h"


 //-------------------------���ܴ�������ˣ�����Ľӿ���
class GlobalAgent_Type0_Vir
{
public:
    GlobalAgent_Type0();

    uint32_t ModeCode;
    virtual void Handle() = 0;
};
//-------------------------CAN���Ӿ�����VisionUartͨѶ������
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

//!----------------���ɣ����۵��������ڱ���̨״̬��ϢͨѶ������
class CloudStateHandler : public GlobalAgent_Can_ViUart
{
public:
    CloudStateHandler(uint32_t mode_name,
                      uint32_t can_commu_id,
                      uint8_t vision_funcword)
        : GlobalAgent_Can_ViUart(mode_name,
                                 can_commu_id,
                                 vision_funcword){};
    //������̨��Ϣ  Cloud State
    float RealAngle[2]; //![Pitch,Yaw] ��ͬ
    float RealMechanical[2];
    //�ϼ�Ҫ����̨��Ϣ Superior Command for Cloud
    float CommandAngle[2];
    
    void Handle();  //���º���
};

void CloudStateHandler::Handle()
{
    RealAngle[1] = Self
}