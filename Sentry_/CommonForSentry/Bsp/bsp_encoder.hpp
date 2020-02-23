


/**
  * @file  bsp_encoder.hpp
  * @brief    RM2020通用编码器
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/20    v0.1
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __BSP_ENCODER_HPP_
#define __BSP_ENCODER_HPP_
#include "tim.h"
// #include "bsp_stddef.h"
//外设宏定义
#define BSP_ENCODER_TIM htim8

extern uint32_t bsp_encoder_PeriodCount;	//设定编码器一圈数值
extern uint32_t bsp_encoder_UpdateTime;		//更新时间戳
extern int32_t bsp_encoder_Value;		//用户使用的编码器值
extern float bsp_encoder_Speed;



void bsp_encoder_Init(uint32_t period_cnt);	//初始化编码器
void bsp_encoder_It();	//溢出中断处理函数
void bsp_encoder_Handle();	//周期性调用此函数以更新编码器
void bsp_encoder_SetValue(int32_t value_to_set);	//设定编码器值



#endif // __BSP_ENCODER_HPP_
