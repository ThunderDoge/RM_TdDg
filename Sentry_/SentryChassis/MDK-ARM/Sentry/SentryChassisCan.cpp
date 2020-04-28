/**
  * @file  SentryChassisCan.cpp
  * @brief      
  * @details    
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryChassisCan.hpp"

void ChassisCanFilterConfig()
{
    
}
/**
* @brief  CAN�����ж�
* @details  ���¶�������жϣ����Զ���CAN�ж��е��ã�����Ҫ�ֶ����,ʹ�õ�ʱ�������ڴ˺������滻��������
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	static CAN_RxHeaderTypeDef bsp_can_Rx;
	uint8_t CAN_RxData[8];
	
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //�ж��жϲ���
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//��ȡCAN����
		motor::CANUpdate(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//�����Ϣ����
		CanRxCpltCallBack_ChassisCommuUpdata(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//���CANͨ����Ϣ����
	}
}



