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

#define __CLOUD_MODE_DEF

///云台模式取值枚举型
enum _cloud_ctrl_mode:uint8_t
{
    save_cloud,
	speed_cloud,
	absolute_cloud,
    relative_cloud,
    absolute_gyro_cloud,
	relative_gyro_cloud,
    absolute_auto_cloud,
	relative_auto_cloud,
	auto_cloud,
};

#define __CHASSIS_MODE_DEF

///底盘模式取值枚举型
enum _chassis_mode:uint8_t
{
    _chassis_speed =1,
    _chassis_location =2,
    _chassis_location_limit_speed =3,
    _chassis_save =0,
};
#endif // !__SENTRY_CTRL_DEF_HPP_

