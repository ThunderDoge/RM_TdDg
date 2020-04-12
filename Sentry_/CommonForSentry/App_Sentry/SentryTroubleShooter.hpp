/* -- Encoding = GB2312 -- */
/**
  * @file  SentryTroubleShooter.hpp
  * @brief    SentryTroubleShooter 哨兵故障自诊断功能，试作型01号
  * @details  使用RGB
  * @author   ThunderDoge
  * @date     2019/12/25
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_DIAG_HPP_
#include "Sentry.hpp"
#include "cmsis_os.h"

extern uint8_t RGB_LoopScript[30];
extern uint16_t RGB_ScriptDuration[30];
extern uint32_t RGB_UsingFrame;
extern uint32_t RGB_FrameTime;

void SentryTroubleShooter_Init(void);
void task_SentryTroubleShooter(void* param);
void CloudRGBAlert_Handle(void);

//Private
#define RGB_CODE_OF(r,g,b) \
    ( (uint8_t) \
        ( \
            ((uint8_t)(r!=0)<<0) ^ \
            ((uint8_t)(g!=0)<<1) ^ \
            ((uint8_t)(b!=0)<<2) \
        ) \
    )   \



void RGB_SetByCode(uint8_t rgb_code);
#endif // !__SENTRY_DIAG_HPP_

