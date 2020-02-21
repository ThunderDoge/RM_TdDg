/**
  * @file  sentry_can_commom.cpp
  * @brief    CANͨѶ������Ϣ
  * @details  
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */    
#include "sentry_can_commom.hpp"

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
 * @brief     CAN����float�Ͳ���
 * ��������float
 * @param     _hcan Ҫ���͵�CAN�ľ��
 * @param     command_id ����ID����SENTRY_CAN_ID
 * @param     argu1 ��һ��float
 * @param     argu2 �ڶ���float
 * @return HAL_StatusTypeDef HAL_OKΪ����
 */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2)
{
    float toSend[2] = {argu1, argu2};
    return SentryCanSend(_hcan, command_id, (uint8_t *)toSend);
}


/**
 * @defgroup Can_TxRx_Functions
 * CAN TxRx Functions CAN���ͺͽ��պ��������еĽ��պ�����ֱ������CAN�ص�StdId��ptrData
 * @{
 */

///CAN���պ���
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    CHASSIS_SUPERIOR_ALL_CanRx(RxHead->StdId, Data);
}



/**
 * @defgroup Can_Rx_Functions
 * CAN���պ�����ͳһ�Ĳ����б�:
 *  StdId   CAN�ص�������StdId
 *  ptrData CAN�ص�����������
 *  @{
 */
/**
 * @brief CAN��������̨״̬��ʹ��ʱֱ����CAN�ص�������������
 * 
 * @param     StdId     CAN�ص�������StdId
 * @param     ptrData   CAN�ص�����������
 */
void UP_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_CLOUD_STATES)
        memcpy(&CanRx.UpCloudPitchYaw, ptrData, 8);
}
void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_CLOUD_STATES)
        memcpy(&CanRx.DownCloudPitchYaw, ptrData, 8);
}
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == CHASSIS_STATES)
	{
        memcpy(&CanRx.Chassis_SpeedLocation, ptrData, 8);	//ֱ�Ӹ��������������������鼴��
	}
}
void CHASSIS_PILLAR_CanRx(uint32_t StdId, uint8_t *ptrData)
{
	if(StdId == CHASSIS_PILLAR)
	{
		CanRx.Pillar_flag = ptrData[0];
	}
}


/**
 * @}
 * End of Group Can_Rx_Functions
 * 
 * 
 * 
 * 
 * @defgroup Can_Rx_Functions
 * CAN���ͺ�����
 * @{
 */


void UP_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, UP_CLOUD_STATES, CanTx.UpCloudPitchYaw[0],
                  CanTx.UpCloudPitchYaw[1]);
}

void DOWN_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, DOWN_CLOUD_STATES,
                  CanTx.DownCloudPitchYaw[0], CanTx.DownCloudPitchYaw[1]);
}
void CHASSIS_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, CHASSIS_STATES,
                  CanTx.Chassis_SpeedLocation[0],
                  CanTx.Chassis_SpeedLocation[1]);
}
void CHASSIS_PILLAR_CanTx()
{
	CanTx.Pillar_flag = CloudEntity.PillarFlag;
	uint8_t pData[8];
	pData[0] = CanTx.Pillar_flag;
	SentryCanSend(&CAN_INTERBOARD, CHASSIS_PILLAR,pData);
}
/**
 * @}
 * End of group: Can_Rx_Functions
 * 
 */

/**
 * @defgroup SuperiorCommands
 * �ϼ�����(SUPERIOR)�㲥���ϼ�����ָ����ȫ����ָ��
 * @{
 */
//����̨����
void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_RELATIVE_CMD)
        memcpy(&CanRx.SuperCon_Relative_PitchYaw, ptrData, 8);
}
void SUPERIOR_UP_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_RELATIVE_CMD,
                  CanTx.SuperCon_Relative_PitchYaw[0],
                  CanTx.SuperCon_Relative_PitchYaw[1]);
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_ABSOLUTE_CMD)
        memcpy(&CanRx.SuperCon_Absolute_PitchYaw, ptrData, 8);
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_ABSOLUTE_CMD,
                  CanTx.SuperCon_Absolute_PitchYaw[0],
                  CanTx.SuperCon_Absolute_PitchYaw[1]);
}
//����̨����
void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_RELATIVE_CMD)
        memcpy(&CanRx.SuperCon_Relative_PitchYaw, ptrData, 8);
}
void SUPERIOR_DOWN_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_RELATIVE_CMD,
                  CanTx.SuperCon_Relative_PitchYaw[0],
                  CanTx.SuperCon_Relative_PitchYaw[1]);
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_ABSOLUTE_CMD)
        memcpy(&CanRx.SuperCon_Absolute_PitchYaw, ptrData, 8);
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_ABSOLUTE_CMD,
                  CanTx.SuperCon_Absolute_PitchYaw[0],
                  CanTx.SuperCon_Absolute_PitchYaw[1]);
}

//��������Ϊ�ṹ������������

/*
void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_FEED)
        CanRx.feed_flag = *ptrData;
}
void UP_FEED_CanTx()
{
    uint8_t data[8];
    data[0] = CanTx.feed_flag;
    SentryCanSend(&CAN_INTERBOARD, UP_FEED, data);
}
void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_FEED)
        CanRx.feed_flag = *ptrData;
}
void DOWN_FEED_CanTx()
{
    uint8_t data[8];
    data[0] = CanTx.feed_flag;
    SentryCanSend(&CAN_INTERBOARD, DOWN_FEED, data);
}
*/

///�����˶�
void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_MOVE)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_speed;
    }
}
///�����˶�
void SUPERIOR_CHASSIS_MOVE_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///����CAN����ָ����� ģʽΪCAN����λ�ÿ��ƣ��ȴ�ִ��
void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location;
    }
}
///����CANָ����Ƶ��̰�λ���˶�
void SUPERIOR_CHASSIS_SET_LOACTION_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///����CANָ����õ��̿���λ��������ǰ��
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location_limit_speed;
    }
}
///����CANָ����õ��̿���λ��������ǰ��
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location_limit_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///����CAN��ȫģʽָ��
void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_SAFE)
    {
        CanRx.SuperiorControlFlags = 0;
        CanRx.SuperCon_ChassisMode = _chassis_save;
    }
}
///����CAN��ȫģʽָ��
void SUPERIOR_SAFE_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_SAFE, NULL);
}
///����ȫ������ָ��ĺ���
void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId < 0x200)
    {
        UP_FEED_CanRx(StdId, ptrData);
        DOWN_FEED_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_MOVE_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(StdId, ptrData);
        SUPERIOR_SAFE_CanRx(StdId, ptrData);
        CanRx.RecvUpdateTime = HAL_GetTick(); //Update Timestamp
    }
}

/**
 * @}
 * End of group: SuperiorCommands
 * 
 * 
 * 
 * @}
 * End of group: Can_TxRx_Functions
 */
