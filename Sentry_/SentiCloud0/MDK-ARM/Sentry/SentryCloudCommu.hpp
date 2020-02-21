/**
 * @file SentryCloudCommu.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __SENTRY_CLOUD_COMMU_HPP_
#define __SENTRY_CLOUD_COMMU_HPP_

///依赖的文件
#include "app_vision.hpp"
#include "SentryCloudVision.hpp"
#include "SentryCloudCan.hpp"
//CAN板间通信定义，如果修CAN口请修改此处宏定义
#define CAN_INTERBOARD hcan2

//此板子使用了视觉串口
#define USE_VISION


extern Sentry_vision_data VisionTx,VisionRx;    ///通信用暂存变量



/// 主任务中周期性发送函数
extern void CloudCanCommuRoutine(void);
extern void CloudVisionRoutine(void);
///储存用结构体



#endif // __SENTRY_CLOUD_COMMU_HPP_




