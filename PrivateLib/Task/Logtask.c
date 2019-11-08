/** 
* @brief    ���ϵͳ����״̬�ĺ���������
* @details  ���ȼ��ϵ͵�һ��������񣬳�ʼ�����֮ǰϵͳ�����ƣ���ɺ����̵ƣ�ͬʱ���Dbus��CAN�����Ƿ�����
* @author   ������
* @date      2019.10
* @version  1.0 
* @par Copyright (c):  RM2020���
* @par ��־
*/

#include "Logtask.h"

//��̬��������
static int8_t Dbus_Offline_Check(void);
static int8_t CAN_Offline_Check(uint8_t CAN_ID);
static int8_t MPU9250_Offline_Check(int8_t ID);

TaskHandle_t Log_Task_Handle = NULL;	/*ϵͳ״̬������*/
SemaphoreHandle_t Dbus_Check_Handle = NULL; /*Dbus���߼���ֵ�ź���*/
SemaphoreHandle_t CAN_Check_Handle = NULL;	/*CAN���߼���ֵ�ź���*/
EventBits_t	Init_Event;

/**
* @brief  ϵͳ����״̬��������
* @details  ��ʾϵͳ����״̬
* @param  ϵͳ����״̬������
* @retval  NULL
*/
void Log_Task(void* pvParameters)
{
	/*ϵͳ���б�־����*/
	EventBits_t Re_Init_Event;
	while(1)
	{
		Re_Init_Event = xEventGroupWaitBits(Inittask_Event_Handle,
																				Init_Event,
																				pdFALSE,
																				pdTRUE,
																				0);						/*��ȡ��ʼ���¼�״̬*/
		//printf("%d\n",Re_Init_Event);
		if((Re_Init_Event & (Init_Event)) == Init_Event)		/*��ʼ���Ѿ����*/
		{
//			if (CAN_Offline_Check(201) == 0 || Dbus_Offline_Check()== 0)			/*CAN,Dbus�������߼��*/
//			{
//				HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_TogglePin(RGB_R_GPIO_Port,RGB_R_Pin); /*״̬��ָʾ*/
//				Music_Play(warningx);
//			}
//			else
			{
				HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);			/*�����豸��������*/
				HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(RGB_G_GPIO_Port,RGB_G_Pin); /*״̬��ָʾ*/
				//printf("CH0:%d CH1:%d CH2:%d CH3:%d Dial:%d\n",Dbus_Data.CH_0,Dbus_Data.CH_1,Dbus_Data.CH_2,Dbus_Data.CH_3,Dbus_Data.Dial);
				//printf("X:%d Left:%d Right:%d\n",Dbus_Data.Mouse.X,Dbus_Data.Mouse.Leftkey,Dbus_Data.Mouse.Rightkey);
				//printf("ROLL:%4f PITCH:%4f YAW:%4f\n",imu.Roll,imu.Pitch,imu.Yaw);
			}
			Music_Handle();		/*����������״̬����*/
		}
		else																								/*��ʼ����û�����*/
		{
			HAL_GPIO_TogglePin(RGB_B_GPIO_Port,RGB_B_Pin); /*״̬��ָʾ*/
			HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
		}
		vTaskDelay(100);
	}
}

/**
* @brief  Dbus���߼��
* @details  ���Dbus�ź��Ƿ�����
* @param  NULL
* @retval  0���� 1����
*/
static int8_t Dbus_Offline_Check(void)
{
	/*Dbus�ź����߼��*/
	static int16_t Offline_Cnt = 0;
	static BaseType_t Dbus_Check = pdPASS;
	Dbus_Check = xSemaphoreTake(Dbus_Check_Handle,0);
//	if (Dbus_Check == pdTRUE)		/*���ݶ�ֵ�ź����������ݼ������*/
//	{
//		return 1;	/*Dbus����*/
//	}
//	else
//	{
//		return 0;	/*Dbus����*/
//	}
	if (Dbus_Check == pdTRUE)		/*���ݶ�ֵ�ź����������ݼ������*/
	{
		Offline_Cnt = 0;
	}
	else
	{
		Offline_Cnt ++;
	}
	//printf("%d\n",Offline_Cnt);
	if (Offline_Cnt >= 5)			/*��ʱ���ж����ߣ��˴���ֵ0.5S*/
	{
		if(Offline_Cnt == 32767) /*��ֹ�������*/
		{
			Offline_Cnt = 5;
		}
		//printf("Dbus offline\n");
		return 0;				/*Dbus����*/
	}
	else 
	{
		//printf("Dbus online\n");
		return 1;				/*Dbus����*/
	}
}

/**
* @brief  CAN�������߼��
* @details  ���CAN������ָ��ID���豸�Ƿ�����
* @param  �豸ID��
* @retval  0���� 1����
*/
static int8_t CAN_Offline_Check(uint8_t CAN_ID)
{
	/*CAN�������߼��*/
	static int16_t Offline_Cnt = 0;
	static BaseType_t CAN_Check = pdPASS;
	CAN_Check = xSemaphoreTake(CAN_Check_Handle,0);
	if (CAN_Check == pdTRUE)		/*���ݶ�ֵ�ź����������ݼ������*/
	{
		Offline_Cnt = 0;
	}
	else
	{
		Offline_Cnt ++;
	}
	//printf("%d\n",Offline_Cnt);
	if (Offline_Cnt >= 5)			/*��ʱ���ж����ߣ��˴���ֵ0.5S*/
	{
		if(Offline_Cnt == 32767) /*��ֹ�������*/
		{
			Offline_Cnt = 5;
		}
		//printf("Dbus offline\n");
		return 0;				/*CAN����*/
	}
	else 
	{
		//printf("Dbus online\n");
		return 1;				/*CAN����*/
	}
}

/**
* @brief  MPU9250���߼��
* @details  ��ȡ9250ID��
* @param  ID ID��
* @retval 1���� 0���� 
*/
static int8_t MPU9250_Offline_Check(int8_t ID)
{
	if(bsp_SPI_MPU9250_Readreg(0X75)!=0X71)
		return 0;				/*MPU9250����*/
	if(bsp_SPI_MPU9250_Readreg(0X75)==0X71)
		return 1;			/*MPU9250����*/
}
