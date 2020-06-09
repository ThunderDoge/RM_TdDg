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
#ifndef __SENTRY_DOWN_CLOUD_COMMU_HPP_
#define __SENTRY_DOWN_CLOUD_COMMU_HPP_

#include "sentry_dwncld_config.h"

///依赖的文件
#include "app_vision.hpp"
#include "SentryCloudVision.hpp"
#include "SentryDownCloudCan.hpp"

//此板子使用了视觉串口
#define USE_VISION

/// 主任务中周期性发送函数
void UpCloudCanCommuRoutine(void);
void CloudVisionTxRoutine(void); ///主逻辑回调函数。向小主机发送一次VisionTx的全部信息。

extern sentry_vision_data VisionTx, VisionRx; ///视觉通信用暂存变量

///回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan); //重定义的CAN中断回调函数
#endif                                                           // __SENTRY_DOWN_CLOUD_COMMU_HPP_
