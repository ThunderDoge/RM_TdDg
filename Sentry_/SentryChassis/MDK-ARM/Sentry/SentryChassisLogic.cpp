/**
 * @file      SentryChassisLogic.cpp
 * @brief     哨兵底盘运行逻辑
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-12
 * @version   1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 */
#include "SentryChassisLogic.hpp"


//模式定义
app_Mode ModeSuperSuperiorControl(NULL, SuperiorControl, NULL);
//app_Mode ModeAutonomousDrive()
app_Mode ModeGlobalSafe(NULL, GlobalSafe, NULL);
app_Mode ModeAutomonus(NULL, Automonus, NULL);
app_Mode ModeChassisTest(NULL,TestChassis,NULL);
//模式指针
app_Mode *CurrentMode = &ModeGlobalSafe;
app_Mode *LastMode = &ModeGlobalSafe;

uint8_t force_mode = 0;
/**
 * @brief 模式选择器。此为所有逻辑起始处
 * 
 */
void ModeSelect(void)
{
    LastMode = CurrentMode;
    switch (force_mode)
    {
    case 3:
        CurrentMode = &ModeChassisTest;
        break;
    case 1:
        CurrentMode = &ModeSuperSuperiorControl;
        break;
    case 2:
        CurrentMode = &ModeAutomonus;
        break;
    case 0:
    default:
        if (GlobalCheckList.IsOfflineList[id_UpCloudConnect] && GlobalCheckList.IsOfflineList[id_DownCloudConnect])
        {
            CurrentMode = &ModeAutomonus;
        }
        else
        {
            CurrentMode = &ModeSuperSuperiorControl;
        }
        break;
    }

    if (LastMode != CurrentMode)
    {
        LastMode->Exit();
        CurrentMode->Enter();
    }
    CurrentMode->Run();
}

void SuperiorControl() //视觉调试
{
    ChassisEntity.ChassisMode = (_chassis_mode)CanRx.SuperCon_ChassisMode;
    switch (CanRx.SuperCon_ChassisMode)
    {
    case _chassis_speed: // 底盘单速度环控制
        ChassisEntity.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location: // 底盘位置环控制
        ChassisEntity.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed: // 底盘位置环限速移动
        ChassisEntity.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
                                                       CanRx.Chassis_SpeedLimit);
        break;
    default:
        ChassisEntity.Safe_Set();
        break;
    }
}

void GlobalSafe()
{
    ChassisEntity.Safe_Set();
}

float rand_target_location;
float reach_target_thres = 10;
/**
 * @brief 自行运动模式
 * 
 */
void Automonus(void)
{
    if (!app_check_IsOffline(id_UpCloudConnect) || !app_check_IsOffline(id_DownCloudConnect)) // 检查有没有上线
    {
        CurrentMode = &ModeSuperSuperiorControl;
        return;
    }

    //	if( GET_DMG_IN_LAST_5s)
    //	{
    //		ENABLE_USE_BUF_POW;
    //	}
    //	else
    //	{
    //		DISABLE_USE_BUF_POW;
    //	}
    //
    //	if( REACHED_LAST_LOCATION )
    //	{
    //		GERERATE_NEXT_LOCATION();
    //	}
    //	CTRL_TO_LOCATION();
    //	ChassisEntity.Safe_Set();
}
float ChassisTestSpeed;
void TestChassis(void)
{
    if(ChassisTestSpeed!=0)
        ChassisEntity.MotorSpeed_Set(ChassisTestSpeed);
    else
        ChassisEntity.Safe_Set();
}

// 这些是撞柱逻辑的变量
int8_t hit_direction; /// 方向。>0表示左边，<0表示右边
int8_t last_hit_direction;
int8_t hit_action_state_indicator = 0;
float hit_speed_setting = 8000; /// 冲击时速度
uint32_t bang_tick;             /// 检测到撞击的时间
float ready_location;
/**
 * @brief 撞击柱子的逻辑
 * 这是一个状态机 hit_action_state_indicator 是状态指示变量
 * 
 * @return return_val 表示撞柱动作状态 1=进行中. 0=完成/无动作 
 * 你可以简单地 while(HitPillarCommand){Delay;} 来使用此函数
 */
int8_t HitPillarCommand()
{
    int8_t return_val = 0;

    if (hit_direction == 0)
    {
        return_val = 0;
        ChassisEntity.Safe_Set();
    }
    else
    {
        return_val = 1;

        if (last_hit_direction != hit_direction)
        {
            hit_action_state_indicator = NO_ORDER;
        }

        if(hit_direction > 0)
            ready_location = RAIL_LEFT_END_MM - LEFT_BOUNCE_READY_LOCATION;
        else // (hit_direction < 0)
            ready_location = RAIL_RIGHT_END_MM + RIGHT_BOUNCE_READY_LOCATION;


        switch (hit_action_state_indicator)
        {
        case NO_ORDER:
            if (fabs(ChassisEntity.RealPosition - ready_location) < reach_target_thres)
                hit_action_state_indicator = MOV_HIT_ACCELE;
            else
            {
                ChassisEntity.MotorSoftLocation_Set(ready_location);
                hit_action_state_indicator = MOV_HIT_PREPARE;
            }

            break;

        case MOV_HIT_PREPARE:
            if (fabs(ChassisEntity.RealPosition - ready_location) < reach_target_thres)
                hit_action_state_indicator = MOV_HIT_ACCELE;
            else
                ChassisEntity.MotorSoftLocation_Set(ready_location);
            break;

        case MOV_HIT_ACCELE:
            
            if ((ChassisEntity.PillarFlag == PILLAR_HIT_LEFT && hit_direction >0)
                || (ChassisEntity.PillarFlag == PILLAR_HIT_RIGHT && hit_direction <0))
            {
                ChassisEntity.Safe_Set();
                bang_tick = HAL_GetTick();
                hit_action_state_indicator = MOV_HIT_BANG;
            }
            else
            {
                ChassisEntity.MotorSpeed_Set(hit_speed_setting * SIGN(hit_direction) );
            }
            break;

        case MOV_HIT_BANG:
            if  ( ((HAL_GetTick() - bang_tick) > 100)   // 首先够时间
                &&((ChassisEntity.PillarFlag != PILLAR_HIT_LEFT && hit_direction >0)    // 左边撞左边
                    || (ChassisEntity.PillarFlag == PILLAR_HIT_RIGHT && hit_direction <0))  // 右边撞右边
                )
            {
                ChassisEntity.MotorSpeed_Set(-hit_speed_setting * SIGN(hit_direction)); // 与MOV_HIT_ACCELE时反向
                hit_action_state_indicator = MOV_HIT_BACK;
            }
            else
            {
                ChassisEntity.DriveWheel.Safe_Set();   //撞击时底盘电机停止
            }
            break;

        case MOV_HIT_BACK:
            if ((hit_direction > 0 && ChassisEntity.RealPosition < ready_location)
            || (hit_direction < 0 && ChassisEntity.RealPosition > ready_location))  // 离开了柱子边缘到 ready_loaction这段区域
            {
                hit_direction = 0;                      // 这个变量置0. 要开始下一次你必须程序设置这个变量
                hit_action_state_indicator = NO_ORDER;  // 回复状态
                ChassisEntity.DriveWheel.Safe_Set();    // 底盘电机停止.你可以进行其他操作.
                return_val = 0;
            }
            else
            {
                ChassisEntity.MotorSpeed_Set(-hit_speed_setting * SIGN(hit_direction)); // 与MOV_HIT_ACCELE时反向，离开柱子
            }
            break;
        }
    }

    return return_val;
}
