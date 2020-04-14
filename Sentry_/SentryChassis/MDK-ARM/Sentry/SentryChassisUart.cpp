/**
 * @file      SentryChassisUart.cpp
 * @brief     
 * @details   
 * @author   ThunderDoge
 * @date      
 * @version   
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */

#include "SentryChassisUart.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 激光测距模块 数据更新
    if(huart == ChassisEntity.RangingLeft.uart_interface)
        bsp_GY53L1_Object_Idle_RxCpltCallback(&ChassisEntity.RangingLeft);
    if(huart == ChassisEntity.RangingRight.uart_interface)
        bsp_GY53L1_Object_Idle_RxCpltCallback(&ChassisEntity.RangingRight);
}


