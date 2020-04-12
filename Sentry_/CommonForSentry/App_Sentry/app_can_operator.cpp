/**
 * @file      app_can_operator.cpp
 * @brief     class CanOperator 用于处理CAN_ID发送接收和处理的类
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
 * @brief Construct a new Can Operator object 创建一个新的CAN通信操作器。每个CAN_ID对应一个操作数。
 * 
 * @param     sentry_can_id     此操作器对应的CAN_ID
 */
CanOperator::CanOperator( uint32_t sentry_can_id, (void)(*pfunc)(uint32_t , uint8_t*) ):
sentry_can_id(sentry_can_id),callback(pfunc),update_time(0)
{}

/**
 * @brief Construct a new Operate object
 * 
 * @param     sentry_can_id     
 * @param     pData 
 */
void CanOperator::Operate(uint8_t *pData)
{
    update_time=HAL_GetTick();
    if(callback!=nullptr && callback!=NULL)
        callback(pData);
}
