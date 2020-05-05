/** 
 * @brief    ÉÚ±øRTOSÈÎÎñ
 * @details  
 * @author   ThunderDoge
 * @date      
 * @version  v0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
 
#ifndef	__TASK_SENT_CHA_HPP_
#define	__TASK_SENT_CHA_HPP_

#include "cmsis_os.h"
#include "bsp_spi.h"
#include "app_imu.h"
#include "app_math.h"
#include "Sentry.hpp"
#include "app_check.h"

void TaskStarter(void);     /// 任务启动器
void RoboInit(void);        /// 硬件初始化
void task_Main(void* param);        /// 主任务
void task_Commu(void* param);       /// 板间通讯任务
void task_OfflineCheck(void* param);    /// 离线检测任务

#endif	//__TASK_SENT_CHA_HPP_