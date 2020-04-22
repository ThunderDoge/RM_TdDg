/**
  * @file      app_check.h
  * @brief     设备离线检测
  * @details   
  * @author   ThunderDoge
  * @date      2020-4-14
  * @version   v0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */
#ifndef __APP_CHECK_H
#define __APP_CHECK_H

#include "stm32f4xx_hal.h"
#include "string.h"

/**
 * @brief 离线设备ID列表。
 * 如果需要新添加设备请添加在这里
 */
typedef enum __app_check_device_id{
    id_UpCloudConnect,
    id_DownCloudConnect,
    id_ChassisConnect,

    id_UpCloudLeftFric,
    id_UpCloudRightFric,
    id_UpCloudYawMotor,
    id_UpCloudPitchMotor,
    id_UpCloudFeedMotor,
	id_UpCloudImu,

    id_DownCloudLeftFric,
    id_DownCloudRightFric,
    id_DownCloudYawMotor,
    id_DownCloudPitchMotor,
    id_DownCloudFeedMotor,
	id_DownCloudImu,

    id_ChassisDriveMotor,
    id_ChassisLazerRangingLeft,
	id_ChassisLazerRangingRight,
	id_ChassisRailEncoder,
    id_ChassisImu,

    id_Dbus,
	
	id_Test,

    DeviceIdEnumCount,  ///< 这个放在最末，指示设备表的长度
}DeviceIdEnum;

/**
 * @brief 设备结构体
 * 
 */
typedef struct __app_check_device_type{
    DeviceIdEnum    DeviceID;            ///< 设备ID
    uint32_t    LastTick;              ///< 指示最后更新时间
    uint32_t*   LastTickPtr;          ///< 更新时间变量指针，如果设置为NULL以外的值，会使LastTick失效，转而使用这个指针作离线判断
    uint32_t    OfflineThreshold;      ///< 离线时间阈值
    uint8_t     LastOfflineState;      ///< 保存上次状态。
    void (*offline_callback)(__app_check_device_type*); ///< 离线处理函数的指针。自动调用
    void (*online_callback)(__app_check_device_type*);  ///< 上线处理函数的指针。自动调用
}app_check_DeviceTypedef;

/**
 * @brief 设备表结构体
 * 
 */
typedef struct __app_check_devicelist_type{
    app_check_DeviceTypedef Device[DeviceIdEnumCount];  ///< 设备数组
    uint8_t IsEnabledList[DeviceIdEnumCount];   ///< 设备启用指示 列表
	uint8_t IsOfflineList[DeviceIdEnumCount];	///< 设备离线指示 列表
}app_check_DeviceListTypedef;


typedef void(*pDeviceCallbackTypedef)(app_check_DeviceTypedef*);    /// 回调函数指针


extern app_check_DeviceListTypedef GlobalCheckList; /// 全局设备表。此文件所有函数都是操作这一设备表。

void app_check_Init(void);  /// 设备表初始化

void app_check_EnableDevice(DeviceIdEnum device_id,uint32_t threshold); /// 启用设备的离线检查，并且设置离线阈值
void app_check_DisableDevice(DeviceIdEnum device_id);   /// 停用设备的离线检查
uint8_t app_check_IsEnabled(DeviceIdEnum device_id);    /// 设备启用/停用状态

void app_check_UpdateTick(DeviceIdEnum device_id);      /// 更新Tick
void app_check_SignDeviceTickTo(DeviceIdEnum device_id,uint32_t* tick_ptr); /// 设定更新时间变量的指针
void app_check_SetDeviceTick(DeviceIdEnum device_id,uint32_t tick); /// 设定 更新时间 的值

uint8_t app_check_IsOffline(DeviceIdEnum device_id);    /// 检查是否离线
void app_check_RefreshList(void);	/// 全部检查一遍是否离线.

void app_check_SignOfflineCallback(DeviceIdEnum device_id,pDeviceCallbackTypedef fptr);   /// 指定上线处理函数
void app_check_SignOnlineCallback(DeviceIdEnum device_id,pDeviceCallbackTypedef fptr);    /// 指定离线处理函数


#endif // __APP_CHECK_H


