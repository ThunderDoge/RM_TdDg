#ifndef __SENTRY_CHASSIS_LOGIC_H_
#define __SENTRY_CHASSIS_LOGIC_H_
#include "SentryChassis.hpp"
#include "SentryChassisCommu.hpp"
//#include "SentryCanCommu.hpp"
//#include "SentryCommu.hpp"
#include "bsp_adc_deal.h"
#include "app_mode.hpp"

enum GlobalModeName
{
    MODE_SAFE = 1,
    MODE_MANUAL_SHOOTING_TEST,
    MODE_VIISON_SHOOTING_TEST,
    MODE_KEYBOARD_TEST,
    MODE_MANUAL_CHASSIS_MOVE,
    MODE_AUTONOMOUS,
};

GlobalModeName GetGlobalMode();
//extern CommandSourceName CommandSource;
extern GlobalModeName RecvCMD;

void ModeSelect(void);  ///模式运转逻辑
void SuperiorControl(); ///上级通讯控制模式
void GlobalSafe();  ///全局安全模式

extern Mode ModeSuperSuperiorControl,ModeGlobalSafe;

#endif // __SENTRY_CHASSIS_LOGIC_H_
