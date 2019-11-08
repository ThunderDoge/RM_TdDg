/** 
* @brief    LED板级支持包
* @details  初始化LED引脚输出状态
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/
#include "bsp_led.h"

/**
* @brief  LED灯状态初始化
* @details  初始化LED灯引脚为低电平
* @param  NULL
* @retval  NULL
*/
void bsp_led_Init(void)
{
	HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
}
