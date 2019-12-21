/**
  * @file  bsp_encoder.cpp
  * @brief    RM2020通用编码器
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/20    v0.1
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "bsp_encoder.hpp"

uint32_t bsp_encoder_PeriodCount;	//设定编码器一圈数值
uint32_t bsp_encoder_UpdateTime;	//更新时间戳
int32_t bsp_encoder_Value;		//用户使用的编码器值

static int32_t soft_period_passed;	//已转过的圈数
/**
 * @brief  初始化编码器
 */
void bsp_encoder_Init(uint32_t period_cnt)
{
    bsp_encoder_PeriodCount  = BSP_ENCODER_TIM.Instance->ARR = period_cnt;
	HAL_TIM_Encoder_Start(&BSP_ENCODER_TIM,TIM_CHANNEL_ALL);
}
/**
 * @brief  溢出中断处理函数
 * @details  将此函数放在BSP_ENCODER_TIM的中断处理中
 */

void bsp_encoder_It()
{
	if( __HAL_TIM_GET_FLAG(&BSP_ENCODER_TIM,TIM_FLAG_UPDATE) ) //处理定时器溢出(更新)中断
	{
		soft_period_passed +=  IS_TIM_COUNTER_MODE(TIM_COUNTERMODE_UP)? 1 : -1  ;	//软圈数增加方向取决于定时器溢出方向
		__HAL_TIM_CLEAR_FLAG(&BSP_ENCODER_TIM,TIM_FLAG_UPDATE);	//清空计数器
	}
}
/**
 * @brief  周期性调用此函数以更新编码器
 */
void bsp_encoder_Handle()
{
	//实时更新你的编码器值
	bsp_encoder_Value  = soft_period_passed*bsp_encoder_PeriodCount + __HAL_TIM_GetCounter(&BSP_ENCODER_TIM);
	bsp_encoder_UpdateTime = HAL_GetTick();
}
/**
 * @brief  用此函数设定编码器值
 * @param[in]  value_to_set 将要设定的值
 */

void bsp_encoder_SetValue(int32_t value_to_set)
{
	soft_period_passed = value_to_set/bsp_encoder_PeriodCount;
	__HAL_TIM_SET_COUNTER(&BSP_ENCODER_TIM, (value_to_set%bsp_encoder_PeriodCount));
}



