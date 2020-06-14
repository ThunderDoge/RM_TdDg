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

extern float RAIL_LEFT_END_MM            ;
extern float RAIL_RIGHT_END_MM           ;

#define LEFT_BOUNCE_READY_LOCATION		200
#define RIGHT_BOUNCE_READY_LOCATION		200
#define LEFT_TOUCH_LOCATION		
#define RIGHT_TOUCH_LOCATION
#define ARMOR_WIDTH
#define ROBOT_WIDTH

typedef enum{
	NO_ORDER = 0,
	MOV_HIT_PREPARE= 1,
	MOV_HIT_ACCELE = 2,
	MOV_HIT_BANG = 3,
	MOV_HIT_BACK = 4,
}HitPillarStateEnum;

void ModeSelect(void);  ///模式选择逻辑

void SuperiorControl(); ///上级通讯控制模式
void GlobalSafe();  ///全局安全模式
void Automonus();	/// 自主运动模式
void TestChassis(void); /// 临时设定的测试模式
int8_t HitPillarCommand();  

extern app_Mode ModeSuperSuperiorControl,ModeGlobalSafe,ModeAutomonus;

#endif // __SENTRY_CHASSIS_LOGIC_H_
