/**
 * @file Sentry.hpp
  * @brief    哨兵总头文件
  * @details  
  * @author   ThunderDoge
  * @date     2020-4-15
  * @version  v1.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  * 
  * v1.0    2020-4-15   
  */
/**
 * @mainpage
 */
#ifndef __SENTRY_HPP_
#define __SENTRY_HPP_


#include "sentry_cloud_config.h"
#include "SentryCloudLogic.hpp"
#include "SentryCloudCommu.hpp"

#if ((defined(__PROJECT_SENTRY_CLOUD_) + defined (__PROJECT_SENTRY_DOWN_CLOUD_) + defined (__PROJECT_SENTRY_CHASSIS_))>1)
    #warning "Multiple sentry project identifier macro defined."
    #endif // SENTRY ID MULTI DEF

#if (!defined (__PROJECT_SENTRY_CLOUD_) && !defined (__PROJECT_SENTRY_DOWN_CLOUD_) && !defined (__PROJECT_SENTRY_CHASSIS_))
    #warning "None of sentry project identifier macro found."
    #endif // SENTRY ID NOT FOUND

#endif // __SENTRY_HPP_
