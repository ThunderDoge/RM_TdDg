/**
 * @file      app_can_operator.hpp
 * @brief     class CanOperator ���ڴ���CAN_ID���ͽ��պʹ������
 * @details
 * @author   ThunderDoge
 * @date     2020-2-27
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
                           Using encoding: gb2312
 */
#ifndef __APP_CAN_OPERATOR_
#define __APP_CAN_OPERATOR_

#include "stm32f4xx_hal.h"
#include <string.h>

class CanOperator /// CANͨ�Ų����� ���ڴ���CAN_ID�Ĵ������
{
public:
    CanOperator( uint32_t sentry_can_id, void(*pfunc)(uint8_t* p) );    ///<���캯��

    void Operate(uint8_t* pData);   ///<ִ�в������Ĳ���

    //״̬����
    uint32_t sentry_can_id; ///<CAN_ID
    uint32_t update_time;   ///<���ִ�в�����ʱ��
private:
    void(*callback)(uint8_t* );
};

#endif // !__APP_CAN_OPERATOR_
