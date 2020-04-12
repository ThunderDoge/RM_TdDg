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

#ifndef __PROJECT_SENTRY_DOWN_CLOUD_     //定义工程标识符__PROJECT_SENTRY_DOWN_CLOUD_
#define __PROJECT_SENTRY_DOWN_CLOUD_
#endif // __PROJECT_SENTRY_DOWN_CLOUD_

///依赖的文件
#include "app_vision.hpp"
#include "SentryDownCloudCan.hpp"
#include "app_sentry_check_device.hpp"

//此板子使用了视觉串口
#define USE_VISION





/// 主任务中周期性发送函数
extern void UpCloudCanCommuRoutine(void);
extern void CloudVisonTxRoutine(void);

extern sentry_vision_data VisionTx,VisionRx;    ///通信用暂存变量

///回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);	//重定义的CAN中断回调函数
#endif // __SENTRY_CLOUD_COMMU_HPP_




