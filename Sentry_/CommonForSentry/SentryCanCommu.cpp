/**
  * @brief    �ڱ�����CANͨ��
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCanCommu.hpp"
#include <cstring>
//ȫ��CAN���ձ���
CanCommuRecv_t CanRecv;
/**
  * @brief  �Զ���CAN���ͺ�������bsp_can
  * @details  
  * @param[in]  _hcan   ���͵�CAN �ľ��
  * @param[in]  command_id  ���CANͨѶ����ID��
  * @param[in]  ptrData ���������� ����8�ֽ�
  * @retval  
  */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, uint8_t *ptrData)
{
    uint32_t MailBox;
    CAN_TxHeaderTypeDef bsp_can_Tx;
    HAL_StatusTypeDef HAL_RESULT;

    //�����������ת��Ϊ��׼CAN֡����
    uint8_t Data[8];
    // Data[0] = (uint8_t)((*(Can_Send_Data+0)>>8));
    // Data[1] = (uint8_t)(*(Can_Send_Data+0)) & 0XFF;
    // Data[2] = (uint8_t)((*(Can_Send_Data+1)>>8));
    // Data[3] = (uint8_t)(*(Can_Send_Data+1)) & 0XFF;
    // Data[4] = (uint8_t)((*(Can_Send_Data+2)>>8));
    // Data[5] = (uint8_t)(*(Can_Send_Data+2)) & 0XFF;
    // Data[6] = (uint8_t)((*(Can_Send_Data+3)>>8));
    // Data[7] = (uint8_t)(*(Can_Send_Data+3)) & 0XFF;
    //ȥ����Ĵ�Сͷ��memcpy��
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
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, float argu1, float argu2)
{
    float toSend[2] = {argu1, argu2};
    return SentryCanSend(_hcan, command_id, (uint8_t *)toSend);
}

void CanRxCpltCallBack_CommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    CanRecv.RecvId = RxHead->StdId;
	CanRecv.Ready_Flag =1;	//���б��CAN���վ�������switch��β��
    switch (RxHead->StdId)
    {
    case SUPERIOR_CHASSIS_MOVE:
        memcpy(&CanRecv.ChassisSpeed, Data, 4);
//        memcpy(&CanRecv.ChassisLocation, Data + 4, 4);
        RecvCMD = MODE_VIISON_SHOOTING_TEST;
        CanRecv.SuperiorControlFlags =_SUPERIOR_CHASSIS_SPEED_SET_;
        CanRecv.Ready_Flag =1;
        break;
    case SUPERIOR_CHASSIS_SET_LOACTION:
        memcpy(&CanRecv.ChassisLocation, Data, 4);
        RecvCMD = MODE_VIISON_SHOOTING_TEST;
        CanRecv.SuperiorControlFlags =_SUPERIOR_CHASSIS_LOACATION_SET_;
        CanRecv.Ready_Flag =1;
        break;
	case SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED:
        memcpy(&CanRecv.ChassisLocation, Data, 4);
        memcpy(&CanRecv.ChassisSpeedLimit, Data + 4, 4);
		RecvCMD = MODE_VIISON_SHOOTING_TEST;
		CanRecv.SuperiorControlFlags = _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_;
		CanRecv.Ready_Flag =1;
		break;
	case UP_FEED:
		CanRecv.UpFeedSpeed = *((float*)Data);
		Self.FeedUp.Freefire_Set(CanRecv.UpFeedSpeed);
    case SUPERIOR_SAFE:
        RecvCMD = MODE_SAFE;     //���ڵ���ȫģʽ
        CommandSource = CMDSRC_CAN; //ָ��ԴΪCANͨ��
        CanRecv.SuperiorControlFlags =_SUPERIOR_OFFLINE_;
        CanRecv.Ready_Flag =1;
        break;
		
	case UP_CLOUD_STATES:
		memcpy(&CanRecv.UpCloudPitchYaw,Data,8);
		break;
    default:
		CanRecv.Ready_Flag =0;	//��Ч�ٱ��CAN��Ч
        break;
    }	
	if(CanRecv.Ready_Flag){	//���ȷʵ������
		CanRecv.RecvUpdateTime = HAL_GetTick();		//����ʱ���
		}
	if(CanRecv.SuperiorControlFlags != _SUPERIOR_OFFLINE_){
		CanRecv.SuperiorUpdateTime = HAL_GetTick();	//����ʱ���
		}
}

HAL_StatusTypeDef ChassisCanCommuRoutine(void)
{
	SentryCanSend(&CAN_INTERBOARD,CHASSIS_STATES,Self.MotorSpeed,Self.MotorSoftLocation);
    uint8_t data[8]={0};
    data[0] = (bsp_ADC1_Sharp_Distance[0] < 13.0f);
    data[1] = (bsp_ADC1_Sharp_Distance[1] < 13.0f);
    SentryCanSend(&CAN_INTERBOARD,CHASSIS_PILLAR,data);
}







