/** 
* @brief    CAN�弶֧�ְ�
* @details  CAN����������ã����ݽ��ս�������
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/
#include "bsp_can.h"

//16λ��С��ת��	������̨ͨѶЭ�����
#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8)|\
                             (((uint16_t)(A) & 0x00ff) << 8))


Chassis_Control_Data_t Chassis_Control_Data;	//��̨���صĿ�����Ϣ

Motor_Data Motor_Yaw_Data; /*3508�����Ϣ*/
Motor_Data Motor_Pitch_Data;
Motor_Data M3508_Data;
Motor_Data GM6020_Data;

Motor_Data Motor_1_Data;
Motor_Data Motor_2_Data;
Motor_Data Motor_3_Data;
Motor_Data Motor_4_Data;


CAN_TxHeaderTypeDef Bsp_CAN1_Tx;   /*Can���߷���*/
CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can���߽���*/
CAN_TxHeaderTypeDef Bsp_CAN2_Tx;   /*Can2���߷���*/
CAN_RxHeaderTypeDef Bsp_CAN2_Rx;		/*Can2���߽���*/

uint8_t CAN1_RxData[8];       /*CAN1���ջ�������*/
uint8_t CAN1_TxData[8]={0};   /*CAN1��������*/
uint8_t CAN2_RxData[8];       /*CAN2���ջ�������*/
uint8_t CAN2_TxData[8]={0};   /*CAN2��������*/

int16_t CAN1_TxData16[4] = {0};
int16_t CAN2_TxData16[4] = {0};

/**
* @brief  CAN�������ó�ʼ��
* @details  ��ʼ���˲���������֡����
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Init(void)
{
	/*CAN�˲�������*/
	CAN_FilterTypeDef CAN_FilterConfig;
	
	CAN_FilterConfig.SlaveStartFilterBank=0;
	CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig.FilterIdHigh = 0X0000;
	CAN_FilterConfig.FilterIdLow = 0X0000;
	CAN_FilterConfig.FilterMaskIdHigh = 0X0000;
	CAN_FilterConfig.FilterMaskIdLow = 0X0000;
	CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterConfig.FilterActivation = ENABLE;
	CAN_FilterConfig.FilterBank = 0;
	#ifdef USE_CAN1
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterConfig);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_Start(&hcan1);
	#endif
	#ifdef USE_CAN2
	CAN_FilterConfig.FilterBank = 14;			//λ��14
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterConfig);
	HAL_CAN_Start(&hcan2);
	#endif
}

/****
*@func  ͨ����can���ͺ���
*@brief ȷ��id �ͷ������� �����ݷ��ͳ�ȥ ע��˴���ڲ�����4��int16_t������ ��8��uint8_t��һ��
*@Para
*@Retal
*@data
*******/


HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff)
{
    static CAN_TxHeaderTypeDef	TxHead;//!�涨���͵�֡��ʽ
		TxHead.RTR=CAN_RTR_DATA;//��׼RTR
    TxHead.IDE=CAN_ID_STD;//��׼ID����չ
		TxHead.DLC = 8;
		TxHead.StdId =id;
    static uint8_t Data[8];//�м����飬���ڸߵ�λ����
		Data[0] = (uint8_t)((*(s16buff+0)>>8));
		Data[1] = (uint8_t)(*(s16buff+0));
		Data[2] = (uint8_t)((*(s16buff+1)>>8));
		Data[3] = (uint8_t)(*(s16buff+1));
		Data[4] = (uint8_t)((*(s16buff+2)>>8));
		Data[5] = (uint8_t)(*(s16buff+2));
		Data[6] = (uint8_t)((*(s16buff+3)>>8));
		Data[7] = (uint8_t)(*(s16buff+3));
        uint32_t FifoLevel;//��ǰ���Ͷ��еĳ���
        HAL_StatusTypeDef result=(HAL_CAN_AddTxMessage(_hcan,&TxHead,Data,&FifoLevel));
	return result;
}


/**
* @brief  ������ݼ��ص�CANx_TxData��
* @details  
* @retval  void
*/

void bsp_CAN_TxLoadAllMotorData(void)
{
	
}



/**
* @brief  CAN��������
* @details  ͨ��CAN���߷��Ϳ�������
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Sendmessage(void)
{
	uint32_t MailBox;
	/*CANA������Ϣ����*/
	//CAN1_TxData[0]=0X0F;
	//CAN1_TxData[1]=0XFF;
	#ifdef USE_CAN1
	HAL_CAN_AddTxMessage(&hcan1, &Bsp_CAN1_Tx, CAN1_TxData, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=3);/*�ȴ��������*/
	#endif
	#ifdef USE_CAN2
	HAL_CAN_AddTxMessage(&hcan2, &Bsp_CAN2_Tx, CAN2_TxData, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=3);/*�ȴ��������*/
	#endif
	//printf("Mailbox:%d\n\n",MailBox); /*���Է����Ƿ�ɹ�*/
}

/** 
  * @brief  ����С��
  * @retval     1 ���
  * @retval     0 С��
  */
static int Check_Endian()
{
	union{
		uint16_t a;
		uint8_t a_char[2];
	}b;
  b.a = 0x1234;
  return (0x12 == b.a_char[0]);
}


/** 
  * @brief ģ�������ֽ���תΪ�����ֽ���(������)
*/
static uint16_t ntohs(uint16_t a)
{
	return Check_Endian() ? a : BigLittleSwap16(a);
}

/**
* @brief  ��̨������Ϣ����
* @details  
* @param[in]  
* @retval  
*/
void Cloud_Control_calculate(uint8_t* data)
{
//	MastercanTick = xTaskGetTickCount();//��ȡ��ǰʱ��
//	CloudOutofContact = 0;
	memcpy(&Chassis_Control_Data, data, 8);
	
//	if(!SuperCapKeyData.LowPowerFlag)//�����е�
//	{
//		if(Chassis_Control_Data.cmd == ChassisAutoReload)//�����Զ���λ
//			InsertCapIndex = 0x07;//�Զ���300w
//		else if(!(Chassis_Control_Data.RecFlags.SuperCapFlag==0 && (InsertCapIndex&0x0f)==0x01))//����̨����Ϊ�����������ݶ��������Զ������򲻹���̨����
//			InsertCapIndex = Chassis_Control_Data.RecFlags.SuperCapFlag;
//		InsertCapIndex |= Chassis_Control_Data.RecFlags.SCIAFlag<<4;//�����Ƿ���������־
//	}else InsertCapIndex = 0;//����û��ǿ�ƹر�
	
	Chassis_Control_Data.speed_x = ntohs(Chassis_Control_Data.speed_x);
	Chassis_Control_Data.speed_y = ntohs(Chassis_Control_Data.speed_y);
	
//	if(Chassis_Control_Data.cmd != ChassisFollowSecondaryCloud)//���̸��������̨��Ҫ���⴦��
		Chassis_Control_Data.speed_z = ntohs(Chassis_Control_Data.speed_z);
//	else{
//		Secondary_Yaw_Position = ntohs(Chassis_Control_Data.speed_z);//�˴�Secondary_Yaw_Position������һ����̨��ƽ�ƹ��Ļ�е��
//		Secondary_Yaw_Angle = Secondary_Yaw_Position*360/4096;
//	}
//	if(!(chassis_angle_set-Yaw_Gyro_Angle<-30 && Chassis_Control_Data.speed_z>0)
//	&& !(chassis_angle_set-Yaw_Gyro_Angle> 30 && Chassis_Control_Data.speed_z<0))//ʵ�ʸ���С��30�Ȳż����ۻ�Ŀ��Ƕ�
//		chassis_angle_set -= (float)Chassis_Control_Data.speed_z/2000;
}

/**
* @brief  ���CAN���ݽ������ṹ��ĺ���
* @details  
* @param[in]  
* @param[in]  
* @retval  
*/
static void bsp_Motor_CAN_Analysis(Motor_Data* motor, uint8_t* Data)
{
	motor->StartUpdataTime = HAL_GetTick();
	motor->LastPosition = motor->RealPosition;
	motor->RealPosition = Data[0]<<8 | Data[1];
	motor->RealSpeed    = Data[2]<<8 | Data[3];  
	motor->RealCurrent  = Data[4]<<8 | Data[5];
	motor->RealTemperature = Data[6];
}

/**
* @brief  CAN���ݽ���
* @details  ����CAN���յ������ݰ������µ��ṹ��
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Analysis(void)
{
	switch(Bsp_CAN1_Rx.StdId)
	{
		case 0x205: bsp_Motor_CAN_Analysis(& Motor_Yaw_Data , CAN1_RxData );	break;
		case 0x206: bsp_Motor_CAN_Analysis(& Motor_Pitch_Data , CAN1_RxData );break;
		case CLOUD_TO_CHASSIS_ID:Cloud_Control_calculate(CAN1_RxData);				break;
	}
	switch(Bsp_CAN2_Rx.StdId )
	{
		case 0x201 : bsp_Motor_CAN_Analysis( & Motor_1_Data , CAN2_RxData );	break;
		case 0x202 : bsp_Motor_CAN_Analysis( & Motor_2_Data , CAN2_RxData  ); break;
		case 0x203 : bsp_Motor_CAN_Analysis( & Motor_3_Data , CAN2_RxData  ); break;
		case 0x204 : bsp_Motor_CAN_Analysis( & Motor_4_Data , CAN2_RxData ); 	break;
	}
}

/**
* @brief  CAN�����ж�
* @details  ���¶�������жϣ���CAN�ж��е���
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static BaseType_t CAN_xHigherPriorityTaskWoken;
	#ifdef USE_CAN1
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)!=0) /*�жϴ���*/
		{
			HAL_CAN_GetRxMessage(&hcan1, 0, &Bsp_CAN1_Rx, CAN1_RxData);	/*��ȡCAN����*/
//			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); /*�ͷ�CAN���ݸ��¶�ֵ�ź���*/
//			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); /*�ͷ�CAN���ݸ��¶�ֵ�ź���*/
 			bsp_CAN_Analysis();

		}
	#endif
	#ifdef USE_CAN2
	if(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0)!=0) /*�жϴ���*/
		{
			HAL_CAN_GetRxMessage(&hcan2, 0, &Bsp_CAN2_Rx, CAN2_RxData);	/*��ȡCAN����*/
//			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); /*�ͷ�CAN���ݸ��¶�ֵ�ź���*/
//			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); /*�ͷ�CAN���ݸ��¶�ֵ�ź���*/
			bsp_CAN_Analysis();
		}
	#endif
}
