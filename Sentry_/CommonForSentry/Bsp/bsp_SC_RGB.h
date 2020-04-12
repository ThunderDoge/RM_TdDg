/**
 * @file bsp_SC_RGB.h
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef BSP_SC_RGB

#define BSP_SC_RGB

#define _USE_SOFTWARE_TIMER    //使用RTOS软件定时器
// #define _USE_HARDWARE_TIMER      //使用硬件定时器

#ifdef _USE_SOFTWARE_TIMER 
#include "cmsis_os.h"
#include "main.h"
#define R_GPIO_PROT RGB1_GPIO_Port
#define R_GPIO_PIN RGB1_Pin
#define G_GPIO_PORT RGB2_GPIO_Port
#define G_GPIO_PIN RGB2_Pin
#define B_GPIO_PROT RGB3_GPIO_Port
#define B_GPIO_PIN RGB3_Pin
#endif

#ifdef _USE_HARDWARE_TIMER
#include "tim.h"
#endif

#ifdef _USE_SOFTWARE_TIMER
void bsp_SC_SoftRGB_Init(uint32_t period);
void bsp_SC_SoftRGB_Set(uint32_t r, uint32_t g, uint32_t b);
void bsp_SC_SoftRGB_SetPeriod(uint32_t period);
#endif // _USE_RTOS_SOFTWARE_TIMER

#ifdef _USE_HARDWARE_TIMER
// void bsp_SC_RGB_Init();
// void bsp_SC_RGB_Set(uint8_t r, uint8_t g, uint8_t b);
#endif // _USE_HARDWARE_TIMER

#endif // BSP_SC_RGB
