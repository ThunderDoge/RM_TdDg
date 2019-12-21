


/**
  * @file  bsp_encoder.hpp
  * @brief    RM2020ͨ�ñ�����
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/20    v0.1
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __BSP_ENCODER_HPP_
#define __BSP_ENCODER_HPP_
#include "tim.h"
//����궨��
#define BSP_ENCODER_TIM htim8

extern uint32_t bsp_encoder_PeriodCount;	//�趨������һȦ��ֵ
extern uint32_t bsp_encoder_UpdateTime;		//����ʱ���
extern int32_t bsp_encoder_Value;		//�û�ʹ�õı�����ֵ


void bsp_encoder_Init(uint32_t period_cnt);	//��ʼ��������
void bsp_encoder_It();	//����жϴ�����
void bsp_encoder_Handle();	//�����Ե��ô˺����Ը��±�����


#endif // __BSP_ENCODER_HPP_
