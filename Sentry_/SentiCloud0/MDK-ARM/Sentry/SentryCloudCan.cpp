//
/**
  * @file SentryCloudCan.cpp
  * @brief    �ڱ���̨CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudCan.hpp"
#include <cstring>



//ȫ��CAN���ձ���
/**
 * @brief ��̨�ã�MCU��CANͨѶ��CAN�ص�����
 * 
 * @param     _hcan ����
 * @param     RxHead 
 * @param     Data 
 */
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    // ����̨����Ҫ�����Լ�����Ϣ
    // UP_CLOUD_STATES_CanRx();
    DOWN_CLOUD_STATES_CanRx(RxHead->StdId, Data);
    CHASSIS_STATES_CanRx(RxHead->StdId, Data);
}



/**
 * @brief �趨CAN�������趨
 * 
 */
void CloudCanFilterConfig()
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
		CanRxCpltCallBack_CloudCommuUpdata(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//���CANͨ����Ϣ����
	}
}

