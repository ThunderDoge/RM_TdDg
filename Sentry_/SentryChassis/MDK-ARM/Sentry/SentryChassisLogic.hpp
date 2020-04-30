/**
 * @file      SentryChassisLogic.hpp
 * @brief     哨兵底盘运行逻辑
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
#include "app_mode.hpp"
#include "app_check.h"


#define LEFT_BOUNCE_READY_DISTANCE
#define RIGHT_BOUNCE_READY_DISTANCE
#define LEFT_TOUCH_DISTANCE
#define RIGHT_TOUCH_DISTANCE
#define ARMOR_WIDTH
#define ROBOT_WIDTH


// enum GlobalModeName
// {
//     MODE_SAFE = 1,
//     MODE_MANUAL_SHOOTING_TEST,
//     MODE_VIISON_SHOOTING_TEST,
//     MODE_KEYBOARD_TEST,
//     MODE_MANUAL_CHASSIS_MOVE,
//     MODE_AUTONOMOUS,
// };

void ModeSelect(void);  ///模式选择逻辑

void SuperiorControl(); ///上级通讯控制模式
void GlobalSafe();  ///全局安全模式
void Automonus();	/// 自主运动模式

extern app_Mode ModeSuperSuperiorControl,ModeGlobalSafe,ModeAutomonus;

#endif // __SENTRY_CHASSIS_LOGIC_H_
