/**
 * @file SentryCloudVision.hpp
  * @brief     云台视觉串口通信
  * @details  提供一个云台与他的小主机妈妈进行UART通讯的所有函数。包含接受到的信息：处理-储存，将要发送的信息：打包，发送。
  * 仅仅依赖于app_vision。注意app_vision。这意味着她不能够主动调用其他的组件的函数。所以你将看到很多的**回调函数**的写法
  * @author   ThunderDoge
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CLOUD_VISION_HPP_
#define __SENTRY_CLOUD_VISION_HPP_

#ifndef __PROJECT_SENTRY_CLOUD_     //定义工程标识符__PROJECT_SENTRY_CLOUD_
#define __PROJECT_SENTRY_CLOUD_
#endif // __PROJECT_SENTRY_CLOUD_


///依赖的文件
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "app_vision.hpp"
#include "bsp_gy53l1.h"
#include "cmsis_os.h"
#include "SentryCloud.hpp"
#include "SentryCloudCan.hpp"


/**
 * @brief 哨兵视觉数据缓存结构体
 */

extern sentry_vision_data VisionTx,VisionRx;    ///储存用结构体

extern QueueHandle_t CloudVision_QueueOfMsgToSend;   /// 待发送消息的队列

void CloudVision_TaskTransmit_Init(void);
void CloudVision_TaskTransmit_Handle(void);

void CloudVision_Transmit(uint8_t _Functionword);
void CloudVision_Transmit_Emergency(uint8_t _Functionword);

extern void CloudVisonTxRoutine(void);  ///主逻辑回调函数。向小主机发送一次VisionTx的全部信息。
void CloudVision_SendFrame(uint8_t funcword,uint8_t* pData); ///阻塞式UART发送，立即向小主机发送一个数据帧。

#endif // __SENTRY_CLOUD_VISION_HPP_

