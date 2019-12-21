/**
  * @file  bsp_encoder.cpp
  * @brief    RM2020ͨ�ñ�����
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/20    v0.1
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "bsp_encoder.hpp"

uint32_t bsp_encoder_PeriodCount;	//�趨������һȦ��ֵ
uint32_t bsp_encoder_UpdateTime;	//����ʱ���
int32_t bsp_encoder_Value;		//�û�ʹ�õı�����ֵ

static int32_t soft_period_passed;	//��ת����Ȧ��
/**
 * @brief  ��ʼ��������
 */
void bsp_encoder_Init(uint32_t period_cnt)
{
    bsp_encoder_PeriodCount  = BSP_ENCODER_TIM.Instance->ARR = period_cnt;
	HAL_TIM_Encoder_Start(&BSP_ENCODER_TIM,TIM_CHANNEL_ALL);
}
/**
 * @brief  ����жϴ�����
 * @details  ���˺�������BSP_ENCODER_TIM���жϴ�����
 */

void bsp_encoder_It()
{
	if( __HAL_TIM_GET_FLAG(&BSP_ENCODER_TIM,TIM_FLAG_UPDATE) ) //����ʱ�����(����)�ж�
	{
		soft_period_passed +=  IS_TIM_COUNTER_MODE(TIM_COUNTERMODE_UP)? 1 : -1  ;	//��Ȧ�����ӷ���ȡ���ڶ�ʱ���������
		__HAL_TIM_CLEAR_FLAG(&BSP_ENCODER_TIM,TIM_FLAG_UPDATE);	//��ռ�����
	}
}
/**
 * @brief  �����Ե��ô˺����Ը��±�����
 */
void bsp_encoder_Handle()
{
	//ʵʱ������ı�����ֵ
	bsp_encoder_Value  = soft_period_passed*bsp_encoder_PeriodCount + __HAL_TIM_GetCounter(&BSP_ENCODER_TIM);
	bsp_encoder_UpdateTime = HAL_GetTick();
}
/**
 * @brief  �ô˺����趨������ֵ
 * @param[in]  value_to_set ��Ҫ�趨��ֵ
 */

void bsp_encoder_SetValue(int32_t value_to_set)
{
	soft_period_passed = value_to_set/bsp_encoder_PeriodCount;
	__HAL_TIM_SET_COUNTER(&BSP_ENCODER_TIM, (value_to_set%bsp_encoder_PeriodCount));
}



