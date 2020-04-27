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

#ifndef __APP_CHECK_LED_H
#define __APP_CHECK_LED_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "app_check.h"

/**
 * @brief LED闪动模式 枚举量
 * 
 */
typedef enum __app_check_led_action_pattern_enum{
    LedPatternSingleIdLoop, ///< 单一ID，循环闪动
    LedActionPatternEnumCount,  ///< 此枚举量放在最后以计算枚举量总数
}LedActionPatternEnum;

/**
 * @brief app_check用 LED控制句柄 结构体
 * 
 */
typedef struct __app_check_LED_HandleTypedef{
    uint16_t    GPIO_PIN_X;     ///< GPIO_Pin_X的值
    GPIO_TypeDef*    GPIOX;     ///< GPIOx的值
    uint8_t     IsActiveHigh;   ///< 是否是高有效
    LedActionPatternEnum     ActionPattern; ///< LED闪动模式
    uint16_t    FlashDuration;              ///< 闪光持续时间
    uint16_t    FlashGap;                   ///< 闪光间隔时间
    uint8_t     FlashCount;                 ///< 闪光次数
    uint16_t    FlashWaitTime;              ///< 两轮闪光中间等待时间
}app_check_LED_HandleTypedef;

typedef __packed struct __uint8_t_priority_unit{
    uint8_t IsEnabled   :1;
    uint8_t Priority    :7;
    uint8_t FlahsCount;
}_led_priority_unit;

/// 全局LED指示
extern _led_priority_unit app_check_LED_PriorityList[DeviceIdEnumCount];


/// LED开灯 
#define __APP_CHECK_LED_ON(led)     HAL_GPIO_WritePin(led->GPIOX, led->GPIO_PIN_X, (GPIO_PinState)led->IsActiveHigh )
/// LED关灯
#define __APP_CHECK_LED_OFF(led)    HAL_GPIO_WritePin(led->GPIOX, led->GPIO_PIN_X, (GPIO_PinState)(!led->IsActiveHigh) )

/// LED句柄初始化
void app_check_LED_Init(app_check_LED_HandleTypedef* led, GPIO_TypeDef*GPIOX, uint16_t GPIO_PIN_X,uint8_t is_active_high );
/// LED设置闪动模式 
void app_check_LED_SetPatternSingleIdLoop(app_check_LED_HandleTypedef* led, uint16_t flash_duration,uint16_t flash_gap);
/// 启动某设备的LED显示
void app_check_LED_EnableDevice(DeviceIdEnum device_id,uint8_t flash_count_of_device,uint8_t priority_of_device);
/// 关闭某设备的LED显示
void app_check_LED_DisableDevice(DeviceIdEnum device_id);
/// 设置闪动的变量：闪光次数和闪光等待时间
void app_check_LED_SetFlash(app_check_LED_HandleTypedef* led, uint8_t flash_count, uint16_t flash_wait_time);

// void app_check_LED_Handler(app_check_LED_HandleTypedef* led);

/// LED任务托管。
void app_check_LED_TaskHandler(app_check_LED_HandleTypedef *led);






#endif // __APP_CHECK_LED_H
