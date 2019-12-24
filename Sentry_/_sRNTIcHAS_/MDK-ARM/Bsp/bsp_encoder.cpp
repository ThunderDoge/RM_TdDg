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
uint32_t bsp_encoder_lastupdate;
int32_t bsp_encoder_Value;		//用户使用的编码器值
uint32_t bsp_encoder_speed_update_time;
float bsp_encoder_Speed;
uint32_t DiffTime = 10;
int32_t value_before_difftime=0;

int32_t soft_period_passed;	//已转过的圈数
/**
 * @brief  初始化编码器
 */
void bsp_encoder_Init(uint32_t period_cnt)
{
    bsp_encoder_PeriodCount  = period_cnt;
	BSP_ENCODER_TIM.Instance->ARR = period_cnt-1;
	__HAL_TIM_ENABLE_IT(& BSP_ENCODER_TIM , TIM_IT_UPDATE );
	HAL_TIM_Encoder_Start(&BSP_ENCODER_TIM,TIM_CHANNEL_ALL);
	bsp_encoder_SetValue(0);
}
/**
 * @brief  溢出中断处理函数
 * @details  将此函数放在BSP_ENCODER_TIM的中断处理中
 */

void bsp_encoder_It()
{
	if( __HAL_TIM_GET_FLAG(&BSP_ENCODER_TIM,TIM_FLAG_UPDATE) ) //处理定时器溢出(更新)中断
	{
		soft_period_passed +=  (__HAL_TIM_IS_TIM_COUNTING_DOWN(&BSP_ENCODER_TIM)? -1 : 1)  ;	//软圈数增加方向取决于定时器溢出方向
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
	#ifdef _CMSIS_OS_H
	bsp_encoder_UpdateTime = xTaskGetTickCount();
	#else
	bsp_encoder_UpdateTime = HAL_GetTick();
	#endif
	
	if(bsp_encoder_lastupdate!=bsp_encoder_UpdateTime){	//确保每TICK只执行一次
		if(bsp_encoder_UpdateTime - bsp_encoder_speed_update_time > DiffTime)	//满足时间则输出
		{
			#ifdef _CMSIS_OS_H					
			bsp_encoder_speed_update_time = xTaskGetTickCount();   	//更新时间
			#else								
			bsp_encoder_speed_update_time = HAL_GetTick();			
			#endif		
			bsp_encoder_Speed =((float)(bsp_encoder_Value - value_before_difftime)) /DiffTime;	//输出速度
			value_before_difftime = bsp_encoder_Value;	//更新算速度区间
		}
		else
		{}
	}
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



