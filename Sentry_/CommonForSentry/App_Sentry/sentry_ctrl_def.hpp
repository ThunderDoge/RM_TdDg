/**
  * @file  sentry_ctrl_def.hpp
  * @brief    
  * @details  
  * @author   ThunderDoge
  * @date     2020-2-21
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __SENTRY_CTRL_DEF_HPP_
#define __SENTRY_CTRL_DEF_HPP_

#include "stm32f4xx_hal.h"

///云台模式取值枚举型
typedef enum _cloud_ctrl_mode:uint8_t
{
	hold_cloud,
    save_cloud,
	speed_cloud,
	absolute_cloud,
    relative_cloud,
    absolute_gyro_cloud,
	// relative_gyro_cloud,
    // absolute_auto_cloud,
	// relative_auto_cloud,
	auto_cloud,
    default_cloud_mode,
}CloudMode_t;
///底盘模式取值枚举型
typedef enum _chassis_mode:uint8_t
{
    _chassis_safe =0,
    _chassis_speed =1,
    _chassis_location =2,
    _chassis_location_limit_speed =3,
    _chassis_default =4,
}ChassisMode_t;

typedef enum __sentry_cloud_shoot_mode:uint8_t{
    ShtStop,
    ShtOnce,
    ShtBurst,
    ShtFree,
}ShootModeEnum_t;

typedef enum __sentry_cloud_dual_single_pitch_ctrl:uint8_t{
	__cloud_main_pitch=0U,
	__cloud_second_pitch=1U,
	__cloud_dual_pitch=2U,
}PitchModeEnum_t;

typedef enum __sentry_cloud_mech_gyro_mode:uint8_t{
    MechCtrl,
    GyroCtrl,
    NoneCtrl,
}MechGyroMode_t;

#endif // !__SENTRY_CTRL_DEF_HPP_

