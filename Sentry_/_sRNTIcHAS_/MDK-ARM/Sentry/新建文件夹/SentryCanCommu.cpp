/**
  * @brief    �ڱ���̨CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCanCommu.hpp"
#include <cstring>
//ȫ��CAN���ձ���
/**
  * @brief  �Զ���CAN���ͺ�������bsp_can
  * @details  
  * @param[in]  _hcan   ���͵�CAN �ľ��
  * @param[in]  command_id  ���CANͨѶ����ID��
  * @param[in]  ptrData ���������� ����8�ֽ�
  * @retval  
  */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData)
{
    uint32_t MailBox;
    CAN_TxHeaderTypeDef bsp_can_Tx;
    HAL_StatusTypeDef HAL_RESULT;

    //�����������ת��Ϊ��׼CAN֡����
    uint8_t Data[8];
    if (ptrData != NULL)
    {
        memcpy(&Data, ptrData, 8);
    }
    else //���ô�����ʱ
    {
        memset(&Data, 0, 8);
    }

    //����CAN֡����
    bsp_can_Tx.StdId = (uint32_t)command_id;
    bsp_can_Tx.RTR = CAN_RTR_DATA;
    bsp_can_Tx.IDE = CAN_ID_STD;
    bsp_can_Tx.DLC = 8;
    HAL_RESULT = HAL_CAN_AddTxMessage(_hcan, &bsp_can_Tx, Data, &MailBox);
#ifndef BSP_CAN_USE_FREERTOS
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
        ; //�ȴ�������ɣ������ʹ��FreeRTOS����Բ���Ҫ���,��Ϊ������ȱ�������Ҫ��ʱ��
#endif

    return HAL_RESULT;
}
/**
  * @brief  ����float�Ͳ����ĺ�����װ
  * @details  
  * @param[in]  
  * @retval  
  */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2)
{
    float toSend[2] = {argu1, argu2};
    return SentryCanSend(_hcan, command_id, (uint8_t *)toSend);
}

