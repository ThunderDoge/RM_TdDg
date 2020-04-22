/**
  * @file      app_check_led.h
  * @brief     app_check LED显示支持包
  * @details   
  * @author   ThunderDoge
  * @date      2020-4-18
  * @version   v0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  * Using encoding: UTF-8
  */
#include "app_check_led.h"

/// 默认闪光延迟时间。单位是ms
#define DEFAULT_WAIT_TIME(led_ptr)    1000

_led_priority_unit app_check_LED_PriorityList[DeviceIdEnumCount] = {0};    /// 全局优先级列表


/**
 * @brief LED句柄初始化
 * 
 * @param     led   LED句柄指针
 * @param     GPIOX     GPIO组号
 * @param     GPIO_PIN_X    GPIO针脚号
 * @param     is_active_high    是否是高电平亮灯。是的打1，不是的打0
 */
void app_check_LED_Init(app_check_LED_HandleTypedef* led, GPIO_TypeDef*GPIOX, uint16_t GPIO_PIN_X,uint8_t is_active_high )
{
    led->IsActiveHigh=is_active_high;
    led->GPIOX = GPIOX;
    led->GPIO_PIN_X = GPIO_PIN_X;
    led->ActionPattern=LedActionPatternEnumCount;
}
/**
 * @brief   设定闪灯模式：LedPatternSingleIdLoop
 * 闪光 flash_count 次，每次亮灯 flash_duration 毫秒，灭灯 flash_gap 毫秒。整个过程时长控制为 flash_period 毫秒。
 * 
 * @param     led               LED句柄指针
 * @param     flash_duration    闪光持续时长
 * @param     flash_gap         闪光间隔时常
 * @param     flash_count       闪光总次数
 * @param     flash_period      整个过程的时长
 */
void app_check_LED_SetPatternSingleIdLoop(app_check_LED_HandleTypedef* led, uint16_t flash_duration,uint16_t flash_gap, uint16_t flash_period)
{
    led->FlashDuration = flash_duration;
    led->FlashGap=flash_gap;
    led->FlashWaitTime=flash_period;
    led->ActionPattern = LedPatternSingleIdLoop;
}
/**
 * @brief 启动某设备的LED显示
 * 
 * @param     device_id 
 * @param     flash_count_of_device 
 * @param     priority_of_device 
 */
void app_check_LED_EnableDevice(DeviceIdEnum device_id, uint8_t flash_count_of_device, uint8_t priority_of_device)
{
    app_check_LED_PriorityList[(uint8_t)device_id].IsEnabled = 1U;
    app_check_LED_PriorityList[(uint8_t)device_id].Priority = (priority_of_device>0X7F ? 0X7F : priority_of_device);
    app_check_LED_PriorityList[(uint8_t)device_id].FlahsCount = flash_count_of_device;
    
}
/**
 * @brief 关闭某设备的LED显示
 * 
 * @param     device_id 
 */
void app_check_LED_DisableDevice(DeviceIdEnum device_id)
{
    app_check_LED_PriorityList[(uint8_t)device_id].IsEnabled = 0;
}
/**
 * @brief 
 * 
 * @param     led   LED句柄指针
 * @param     flash_count 闪动次数
 */
void app_check_LED_SetFlash(app_check_LED_HandleTypedef *led, uint8_t flash_count, uint16_t flash_wait_time)
{
    led->ActionPattern = LedPatternSingleIdLoop;
    led->FlashCount = flash_count;
    led->FlashWaitTime = flash_wait_time;
}

/**
 * @brief LED操作托管函数。在 app_check_LED_TaskHandler 中被调用
 * 
 * @param     led 
 */
static void app_check_LED_Handler(app_check_LED_HandleTypedef *led)
{
    switch (led->ActionPattern)
    {
    case LedPatternSingleIdLoop :
        for(int i=0;i<led->FlashCount;i++)
        {
            __APP_CHECK_LED_ON(led);
            osDelay(led->FlashDuration);
            __APP_CHECK_LED_OFF(led);
            osDelay(led->FlashGap);
        }
        osDelay( led->FlashWaitTime );
        break;
    default:
        __APP_CHECK_LED_ON(led);
        break;
    }
}
/**
 * @brief   LED句柄在任务中托管
 * 你需要在你的程序中另建一任务，以调用此函数。否则会阻塞其他任务。
 * @param     led LED句柄指针
 */
void app_check_LED_TaskHandler(app_check_LED_HandleTypedef *led)
{
    // 寻找最高优先级的设备
    int max_priority_i=-1;
    int max_priority=-1;
    for(int i=0;i<DeviceIdEnumCount;i++)
    {
        if(app_check_LED_PriorityList[i].IsEnabled && app_check_LED_PriorityList[i].Priority>max_priority)
        {
            max_priority=app_check_LED_PriorityList[i].Priority;
            max_priority_i=i;
        }
    }
    // 
    if(max_priority_i != -1)
    {
        // 第一次计算的 DEFAULT_PERIOD_TIME 是错误的,因为 FlahsCount 未写入
        app_check_LED_SetFlash(led, app_check_LED_PriorityList[max_priority_i].FlahsCount, DEFAULT_WAIT_TIME );
        app_check_LED_Handler(led);
    }
    else
    {
        __APP_CHECK_LED_ON(led);
    }
}

#undef DEFAULT_PERIOD_TIME

