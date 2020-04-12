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

void TaskStarter(void);     /// ?????
void RoboInit(void);        /// ?????
void task_Main(void* param);        /// ???
void task_Commu(void* param);       /// ??????
void task_OfflineCheck(void* param);    /// ??????

#endif	//__TASK_SENT_CHA_HPP_
