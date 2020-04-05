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
#include "app_vision.hpp"
#include "SentryCloudCan.hpp"


/**
 * @brief 哨兵视觉数据缓存结构体
 * 
 */

extern sentry_vision_data VisionTx,VisionRx;    ///储存用结构体

extern void CloudVisonTxRoutine(void);  ///主逻辑回调函数。向小主机发送一次VisionTx的全部信息。
void CloudVisionSendFrame(uint8_t funcword,uint8_t* pData); ///阻塞式UART发送，立即向小主机发送一个数据帧。

#endif // __SENTRY_CLOUD_VISION_HPP_












/**
 * @brief 视觉传输数据解析结构体
 * @addtogroup Sentry_Vision
 */
// struct Sentry_vision_data
// {
//     uint8_t Frame_header = FRAME_HEADER_DATA;
//     uint8_t Frame_end = FRAME_END_DATA;
//     uint8_t Function_word; ///<数据帧功能字
//     ///底盘数据
//     float Vx;         ///<底盘X轴速度
//     float Vy;         ///<底盘Y轴速度
//     float Px;         ///<底盘X轴路程
//     float Py;         ///<底盘Y轴路程
//     float SpeedLimit; ///<底盘限速
//     uint8_t pillar_flag;
//     uint8_t chassis_mode;
//     ///云台数据
//     float Yaw;          ///<Yaw轴角度
// 	float YawSoft;
//     float Pitch;        ///<Pitch轴角度
//     uint8_t Cloud_mode; ///<云台模式
//     uint8_t cloud_ctrl_mode;
//     ///射击数据
//     uint8_t Shoot_mode; ///<射击模式
//     float Shoot_speed;  ///<射击速度
//     uint8_t Shoot_freq; ///<射击频率
//     ///数据标志
//     uint32_t UpdateTime;
//     ///日志系统使用
//     uint8_t Error_code = 0;          ///<错误代码
//     int16_t CAN1_motorlist = 0xffff; ///<CAN1电机列表
//     int16_t CAN2_motorlist = 0xffff; ///<CAN2电机列表
// };

///ROBOT_ERR 的错误码列表
