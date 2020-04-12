/**
 * @file      app_can_operator.hpp
 * @brief     class CanOperator 用于处理CAN_ID发送接收和处理的类
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

class CanOperator /// CAN通信操作器 用于处理CAN_ID的处理的类
{
public:
    CanOperator( uint32_t sentry_can_id, void(*pfunc)(uint8_t* p) );    ///<构造函数

    void Operate(uint8_t* pData);   ///<执行操作器的操作

    //状态变量
    uint32_t sentry_can_id; ///<CAN_ID
    uint32_t update_time;   ///<最近执行操作的时间
private:
    void(*callback)(uint8_t* );
};

#endif // !__APP_CAN_OPERATOR_
