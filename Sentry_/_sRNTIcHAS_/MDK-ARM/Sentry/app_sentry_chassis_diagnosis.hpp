/**
  * @file      app_sentry_chassis_diagnosis.hpp
  * @brief     ÉÚ±øµ×ÅÌ×ÔÕï¶Ï¡£
  * @details   
  * @author    ThunderDoge
  * @date      2020-2-24
  * @version   0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */
#ifndef __APP_SENTRY_CHASSIS_DIAGNOSIS_HPP_
#define __APP_SENTRY_CHASSIS_DIAGNOSIS_HPP_

#include "stm32f4xx.h"
#include "sentry_config.hpp"

#define APP_SENTRY_CHASSIS_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE 20;

extern uint8_t app_sentry_chassis_diag_OfflineList[APP_SENTRY_CHASSIS_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE];

void app_sentry_chassis_diag_Init();
void app_sentry_chassis_diag_Handle();

#endif // __APP_SENTRY_CHASSIS_DIAGNOSIS_HPP_
 