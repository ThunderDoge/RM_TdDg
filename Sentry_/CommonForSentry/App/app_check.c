/**
 * @file      app_check.c
 * @brief     设备离线检测
 * @details   重写原来的app_check_device，采用纯c
 * 
 * @author   ThunderDoge
 * @date      2020-4-14
 * @version   
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 */

#include "app_check.h"

#define DEVICE_OF(id)   GlobalCheckList.Device[((uint32_t)id)]
#define ENABLE_OF(id)   GlobalCheckList.IsEnabledList[((uint32_t)id)]
#define OFFLINE_OF(id)	GlobalCheckList.IsOfflineList[((uint32_t)id)]

app_check_DeviceListTypedef GlobalCheckList;

void app_check_Init(void)
{
    for(uint32_t i=0;i<DeviceIdEnumCount;i++)
    {
        ENABLE_OF(i) = 0;
		OFFLINE_OF(i) = 1;
        DEVICE_OF(i).OfflineThreshold=100U;
        DEVICE_OF(i).LastTick=0U;
        DEVICE_OF(i).online_callback = NULL;
        DEVICE_OF(i).offline_callback = NULL;
    }
}

void app_check_EnableDevice(DeviceIdEnum device_id,uint32_t threshold)
{
    ENABLE_OF(device_id) = 1;
    DEVICE_OF(device_id).OfflineThreshold = threshold;
}
void app_check_DisableDevice(DeviceIdEnum device_id)
{
    ENABLE_OF(device_id) = 0;
}
uint8_t app_check_IsEnabled(DeviceIdEnum device_id)
{
    return ENABLE_OF(device_id);
}
/**
 * @brief   更新设备的Tick
 * 
 * @param     device_id 
 */
void app_check_UpdateTick(DeviceIdEnum device_id)
{
    DEVICE_OF(device_id).LastTick = HAL_GetTick();
}
/**
 * @brief       设定更新时间变量的指针
 * 
 * @param     device_id 
 * @param     tick_ptr 更新时间变量的指针
 */
void app_check_SignDeviceTickTo(DeviceIdEnum device_id,uint32_t* tick_ptr)
{
    DEVICE_OF(device_id).LastTickPtr = tick_ptr;
}
/**
 * @brief   设定更新时间的值
 * 
 * @param     device_id 
 * @param     tick 
 */
void app_check_SetDeviceTick(DeviceIdEnum device_id,uint32_t tick)
{
    DEVICE_OF(device_id).LastTick = tick;
}

/**
 * @brief       刷新设备的离线状态，返回状态。并且运行回调函数（如果指定）
 * 
 * @param     device_id 
 * @return uint8_t 
 */
uint8_t app_check_IsOffline(DeviceIdEnum device_id)
{
    uint32_t tick_now = HAL_GetTick();
    uint8_t is_offline;
	
	// 检查是否启用
	if(!GlobalCheckList.IsEnabledList[device_id])
		return 2;
    
    // 检查是否指定了更新时间变量的指针
    if(DEVICE_OF(device_id).LastTickPtr!=NULL)  
        // 指定了则根据指针判断
        {is_offline = (tick_now - *DEVICE_OF(device_id).LastTickPtr > DEVICE_OF(device_id).OfflineThreshold);}
    else
        // 未指定则根据结构体内变量判断
        {is_offline = (tick_now - DEVICE_OF(device_id).LastTick > DEVICE_OF(device_id).OfflineThreshold);}

    DEVICE_OF(device_id).LastOfflineState = is_offline; // 保存状态;
	OFFLINE_OF(device_id) = is_offline;

    if(is_offline)  // 根据离线状态运行离线处理函数/上线处理函数
    {
        if( DEVICE_OF(device_id).offline_callback != NULL )
        {
            DEVICE_OF(device_id).offline_callback();
        }
    }
    else
    {
        if( DEVICE_OF(device_id).online_callback != NULL )
        {
            DEVICE_OF(device_id).online_callback();
        }
    }
    
    return is_offline;  // 返回离线状态
}

void app_check_RefreshList(void)
{
	for(int i=0;i<DeviceIdEnumCount;i++)
	{
		if(GlobalCheckList.IsEnabledList[i])
			app_check_IsOffline( (DeviceIdEnum) i );
	}
}

void app_check_SignOfflineCallback(DeviceIdEnum device_id,void(*fptr)(void))
{
    DEVICE_OF(device_id).offline_callback = fptr;
}
void app_check_SignOnlineCallback(DeviceIdEnum device_id,void(*fptr)(void))
{
    DEVICE_OF(device_id).online_callback = fptr;
}


#undef DEVICE_OF
#undef ENABLE_OF
#undef OFFLINE_OF
