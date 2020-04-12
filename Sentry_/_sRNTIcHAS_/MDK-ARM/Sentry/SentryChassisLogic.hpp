/**
 * @file      SentryChassisLogic.hpp
 * @brief     �ڱ����������߼�
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-12
 * @version   1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 */

#ifndef __SENTRY_CHASSIS_LOGIC_H_
#define __SENTRY_CHASSIS_LOGIC_H_

#include "sentry_ctrl_def.hpp"
#include "SentryChassis.hpp"
#include "SentryChassisCommu.hpp"
#include "bsp_adc_deal.h"
#include "app_mode.hpp"
#include "app_sentry_check_device.hpp"


// enum GlobalModeName
// {
//     MODE_SAFE = 1,
//     MODE_MANUAL_SHOOTING_TEST,
//     MODE_VIISON_SHOOTING_TEST,
//     MODE_KEYBOARD_TEST,
//     MODE_MANUAL_CHASSIS_MOVE,
//     MODE_AUTONOMOUS,
// };

void ModeSelect(void);  ///ģʽѡ���߼�

void SuperiorControl(); ///�ϼ�ͨѶ����ģʽ
void GlobalSafe();  ///ȫ�ְ�ȫģʽ

extern app_Mode ModeSuperSuperiorControl,ModeGlobalSafe;

#endif // __SENTRY_CHASSIS_LOGIC_H_
