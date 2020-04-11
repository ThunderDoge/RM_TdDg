/**
  * @file      app_judge_cmd_id.h
  * @brief     裁判系统 命令字 集合
  * @details   
  * @author   ThunderDoge
  * @date      2020-4-9
  * @version   v0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */

#ifndef __APP_JUDGE_CMD_ID_H
#define __APP_JUDGE_CMD_ID_H

#include "stm32f4xx.h"


enum __judge_uart_cmd_id{
    // 接收 命令码
    JUD_UART_GAME_STATUS = 0x1U,    // 比赛状态数据，1Hz 周期发送
    JUD_UART_GAME_RESULT = 0x2U,    // 比赛结果数据，比赛结束后发送 
    JUD_UART_HP = 0x3U,             // 比赛机器人血量数据，1Hz 周期发送 
    JUD_UART_GAME_EVENT =0x101U,    // 场地事件数据，事件改变后发送 
    JUD_UART_SUPPLY_STATION = 0X102U,   // 场地补给站动作标识数据，动作改变后发送 
    JUD_UART_WARNING = 0X104U,          // 裁判警告数据，警告发生后发送 
    JUD_UART_SELF_STATES = 0X201U,      // 机器人状态数据，10Hz 周期发送 
    JUD_UART_GUN_CHASSIS_HEAT = 0x202U, // 实时功率热量数据，50Hz 周期发送 
    JUD_UART_LOCATION = 0X203U,     // 机器人位置数据，10Hz 发送 
    JUD_UART_SELF_BUFF = 0x204U,    // 机器人增益数据，增益状态改变后发送 
    JUD_UART_TAKING_DMG = 0x206U,   // 伤害状态数据，伤害发生后发送 
    JUD_UART_SHOOTING = 0x207U,     // 实时射击数据，子弹发射后发送
    JUD_UART_AMMO_LEFT = 0x208U,    // 子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz 周期发送
    // 发送 命令码
    JUD_UART_USER = 0x301U,     // 机器人间交互数据，发送方触发发送，上限 10Hz
    JUD_UART_SUPPLY_REQ = 0X103U,   
};

enum __JUD_UART_GAME_STATUS_CurrentStage{
    __JUD_UART_GAME_STATUS_CurrentStage_PREPARE=1,
    __JUD_UART_GAME_STATUS_CurrentStage_SELFCHECK=2,
    __JUD_UART_GAME_STATUS_CurrentStage_5SEC=3,
    __JUD_UART_GAME_STATUS_CurrentStage_ENGANGE=4,
    __JUD_UART_GAME_STATUS_CurrentStage_OVER=5,
};


#endif // __APP_JUDGE_CMD_ID_H
