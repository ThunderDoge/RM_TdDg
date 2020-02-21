/** 
 * @file    bsp_can.hpp
* @brief    CAN�弶֧�ְ�
* @details  CAN����������ã����ݽ��ս�������
* @author   Evan-GH
* @date      2019.11
* @version  1.8
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "stm32f4xx.h"
#include "can.h"
#include "bsp_motor.hpp"
#include "task_SentiCloud.hpp"
#include "SentryCanCommu.hpp"




//�������뿪�غ궨�壬����Ҫ�����ľ�ע�͵���غ궨��
#define	BSP_CAN_USE_FREERTOS
//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_CAN_USE_CAN1					hcan1
#define BSP_CAN_USE_CAN2					hcan2

void bsp_can_Init(void);	//CAN���߳�ʼ������
HAL_StatusTypeDef bsp_can_Sendmessage(CAN_HandleTypeDef* hcan,int16_t StdId,int16_t* Can_Send_Data);	//CAN�������ݷ��ͺ���
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);	//�ض����CAN�жϻص�����
#endif
