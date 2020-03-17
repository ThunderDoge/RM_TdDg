/**
 * @file      app_sentry_check_device.h
 * @brief     离线检测功能，云台底盘公用文件。
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#ifndef __APP_SENTRY)CHECK_DEVICE_H_

//依赖的文件
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

//使用设备的宏定义
// #define APP_ALARM_USE_OLED   //使用OLED屏幕
// #define APP_ALARM_USE_BEEP   //使用蜂鸣器
// #define APP_ALARM_USE_LED    //使用LED灯



/**
 * @brief 设备对象ID(告警ID) 结构体
 */
typedef enum:uint8_t
{
    UpCloudConnect  =   0,
    DownCloudConnectDevice,
    ChassisConnectDevice,

    UpCloudLeftFricDevice,
    UpCloudRightFricDevice,
    UpCloudYawMotorDevice,
    UpCloudPitchMotorDevice,
    UpCloudFeedMotorDevice,

    DownCloudLeftFricDevice,
    DownCloudRightFricDevice,
    DownCloudYawMotorDevice,
    DownCloudPitchMotorDevice,
    DownCloudFeedMotorDevice,

    ChassisDriveMotorDevice,
    ChassisPillarDetectDevice,
    DbusDevice,

	CheckDeviceID_EnumLength,  //这一项放在最后，以指示ID项数
}CheckDeviceID_Enum;



/**
 * @brief 设备对象报警优先级
 */
typedef enum 
{
  PriorityLow             = -2,          ///< priority: low
  PriorityBelowNormal     = -1,          ///< priority: below normal
  PriorityNormal          =  0,          ///< priority: normal (default)
  PriorityAboveNormal     = +1,          ///< priority: above normal
  PriorityHigh            = +2,          ///< priority: high
  PriorityHighest         = +3,          ///< priority: highest
}AlarmPriority_Enum;



/**
 * @brief 设备对象 结构体
 */
typedef __packed struct cdt
{
	CheckDeviceID_Enum      id;						/* 器件ID,为防止ID冲突，将所有需检测ID放入CheckDeviceID_Enum变量中 */
	uint32_t                lastTick;            /* 上次在线时间 */
	uint16_t                maxAllowTime;	        /* 最大允许时长 */
	AlarmPriority_Enum      priority;		        /* 优先级 */
	uint8_t                 isOffline : 1 ;           /* 器件在线状态*/
    uint8_t(is_offline_func)(void);                 //离线状态检查函数
}CheckDevice_Type;



/**
 * @brief 设备对象数组，
 */
typedef struct  
{
	CheckDevice_Type* deviceArry[CheckDeviceID_EnumLength]; /* 设备数组 */
	int8_t checkDeviceNum;				       /* 被测设备的数量，运行时.初始化时确定。 */
}CheckDeviceArry_Type;

///初始化设备离线检测各项功能
void CheckDevice_Init(void);

///初始化设备结构体
void CheckDevice_Type_Init(CheckDevice_Type* t);

///将一个设备参数对象 加入设备检测数组中
HAL_StatusTypeDef CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

///通用离线检测任务代码。可以在需要的时候添加到自己的Task实现之中。请注意需要的宏定义。
void CheckDevice_TaskHandler(void);

#endif // !__APP_SENTRY)CHECK_DEVICE_H_