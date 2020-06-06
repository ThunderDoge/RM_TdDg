/**
 * @file      SentryChassisUart.hpp
 * @brief     
 * @details   
 * @author   ThunderDoge
 * @date      
 * @version   
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#ifndef __SENTRY_CHASSIS_UART_HPP 
#define __SENTRY_CHASSIS_UART_HPP

#include "SentryChassis.hpp"
#include "bsp_judgement.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif // __SENTRY_CHASSIS_UART_HPP


