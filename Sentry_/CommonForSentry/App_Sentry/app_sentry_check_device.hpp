/**
 * @file      app_sentry_check_device.hpp
 * @brief     离线检测功能，云台底盘公用文件。
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#ifndef __APP_SENTRY_CHECK_DEVICE_H_
#define __APP_SENTRY_CHECK_DEVICE_H_

//依赖的文件
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "sentry_can_commom.hpp"

//DEBUG用宏定义
#define __APP_CHECK_DEVICE_DEBUG

//使用设备的宏定义
// #define APP_ALARM_USE_OLED   //使用OLED屏幕
// #define APP_ALARM_USE_BEEP   //使用蜂鸣器
// #define APP_ALARM_USE_LED    //使用LED灯

//-----------------------\begin 相关结构体/枚举型定义------------------------

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
	UpCloudImuDevice,

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
typedef enum :int8_t
{
  PriorityLow             = -2,          ///< priority: low
  PriorityBelowNormal     = -1,          ///< priority: below normal
  PriorityNormal          =  0,          ///< priority: normal (default)
  PriorityAboveNormal     = +1,          ///< priority: above normal
  PriorityHigh            = +2,          ///< priority: high
  PriorityHighest         = +3,          ///< priority: highest
}AlarmPriority_Enum;


//下文将使用的定义
struct CheckDevice_Type;
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* self);
void Default_CheckDevice_UpdateHookFunc(CheckDevice_Type* self);


/**
 * @brief 设备对象 结构体
 */
struct CheckDevice_Type
{
	CheckDevice_Type(															///< 构造函数
		CheckDeviceID_Enum  id,
        uint16_t            allow_time,
		uint8_t             (*ptr_is_offline_func)(void)                                = NULL,
		AlarmPriority_Enum  pri                                                         = PriorityNormal,
		void                (*ptr_state_changed_callback_func)(CheckDevice_Type*self)   = Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask,
		void                (*ptr_update_hook_func)(CheckDevice_Type*self)              = Default_CheckDevice_UpdateHookFunc
		);
	CheckDeviceID_Enum      id              = CheckDeviceID_EnumLength;						/** 器件ID,为防止ID冲突，将所有需检测ID放入 @see CheckDeviceID_Enum 变量中 */
	uint32_t                lastTick        = 0;            /* 上次在线时间 */
	uint16_t                maxAllowTime    = 100;	        /* 最大允许时长 */
	AlarmPriority_Enum      priority        = PriorityNormal;		        /* 优先级 */
	uint8_t			        alarm_enabled   = 1;		// 离线报警已使能
	// uint8_t                 is_offline      = 0 ;           /* 器件在线状态*/
    uint8_t                 is_change_reported = 0;       // 是否已发送告警信息
    uint8_t(*is_offline_func)(void)         = NULL;                //离线状态检查函数
	void (*state_changed_callback_func)(CheckDevice_Type* self);	// 离线回调函数。检测到离线之后调用此函数。
	void (*update_hook_func)(CheckDevice_Type* self);		// 状态更新钩子函数。在设备更新时调用。
};


/**
 * @brief 设备对象数组，
 */
typedef struct  
{
	CheckDevice_Type* deviceArry[CheckDeviceID_EnumLength]; /* 设备数组 */
	int8_t checkDeviceNum;				       /* 被测设备的数量，运行时.初始化时确定。 */
}CheckDeviceArry_Type;

//-----------------------\end 相关结构体/枚举型定义------------------------

//-----------------------------\begin 相关全局变量-------------------------------


extern QueueHandle_t QueueOfflineDeviceToCommuTask;     /// 已离线设备列表 发送给通信函数处理
extern CheckDeviceArry_Type CheckDeviceArry;			/// 本机包含的设备的列表
extern uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ];    /// 哨兵所有设备的离线状态


//-----------------------------\end 相关全局变量-------------------------------

//----------------------------- 对外接口函数-------------------------------

///初始化设备离线检测各项功能
void app_sentry_CheckDevice_Init(void);


/////初始化设备结构体
//void app_sentry_CheckDevice_Type_Init(CheckDevice_Type* device);


/////初始化设备离线检测各项功能，并加入设备检测数组中
//void app_sentry_CheckDevice_Type_Init_AddToArray(	CheckDevice_Type* 	device,
//										CheckDeviceID_Enum 	id,
//										uint16_t 			max_allow_time,
//										AlarmPriority_Enum 	priority,
//										FunctionalState		enable_alarm,
//										uint8_t (*ptr_is_offline_func)(void),
//										void (*ptr_update_hook_func)(CheckDevice_Type*self) );



///将一个设备参数对象 加入设备检测数组中
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

///刷新错误列表。通过重新将所有的设备发送到 QueueOfflineDeviceToCommuTask 来实现
void app_sentry_CheckDevice_CheckAllDevice(void);

///通用离线检测任务代码。您的主逻辑Task调用。请注意需要的宏定义。
void app_sentry_CheckDevice_TaskHandler(void);

///您的通信函数调用的 错误设备处理函数。它将处理 @see QueueOfflineDeviceToCommuTask
void app_sentry_CheckDevice_CommuTaskCallback(void);

///CAN 接收函数. 在CAN接收回调函数中调用
void app_sentry_CheckDevice_CanRxCallBack(uint8_t* pdata);

#endif // !__APP_SENTRY_CHECK_DEVICE_H_


