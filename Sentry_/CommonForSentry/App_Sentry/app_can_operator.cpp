/**
 * @file      app_can_operator.cpp
 * @brief     class CanOperator ���ڴ���CAN_ID���ͽ��պʹ������
 * @details   
 * @author   ThunderDoge
 * @date     2020-2-27
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "app_can_operator.hpp"
#include <string.h>

/**
 * @brief       �����յ���ID�ţ����Դ������������ɹ�����1��������ʱ���
 * 
 * @param     cmd_id �������ڴ�����Id��
 * @return uint8_t  1:Operator Trigged; 0:Operator NOT Trigged;
 */
uint8_t CanOperator::TryTrigger(uint32_t cmd_id)
{
    if (cmd_id == TriggerId)
    {
        // Operator Trigged
        if(pReadData[0]!=nullptr && pWriteData[0]!=nullptr)
            memcpy(pWriteData[0], pReadData[0], DataLength[0]);
        if(pReadData[1]!=nullptr && pWriteData[1]!=nullptr)
            memcpy(pWriteData[1], pReadData[1], DataLength[1]);
        TriggedTimestamp = HAL_GetTick();
        if (TrigCallBack != nullptr && TrigCallBack != NULL)
        {
            TrigCallBack(this);
        }
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief   �������͡�ͬ�������ʱ������ص��������û��Զ��壬������ṩ�����ص�����
 * 
 */
void CanOperator::ForceTrigger()
{
        if(pReadData[0]!=nullptr && pWriteData[0]!=nullptr)
            memcpy(pWriteData[0], pReadData[0], DataLength[0]);
        if(pReadData[1]!=nullptr && pWriteData[1]!=nullptr)
            memcpy(pWriteData[1], pReadData[1], DataLength[1]);
        TriggedTimestamp = HAL_GetTick();
        if (TrigCallBack != nullptr && TrigCallBack != NULL)
        {
            TrigCallBack(this);
        }
}
void app_can_operator_StdSendCmdCallBack(CanOperator* self);
{
    uint8_t pData[4];
}

