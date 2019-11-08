/** 
* @brief    ϵͳ��������
* @details  ����CAN,Dbus���ݣ����Ƶ��ת��
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/
#include "Controltask.h"

TaskHandle_t Control_Task_Handle = NULL;	/*����������*/
SemaphoreHandle_t Dbus_Update_Handle = NULL; /*Dbus��ֵ�����±���*/
SemaphoreHandle_t CAN_Update_Handle = NULL; /*CAN��ֵ�����±���*/
 
//PITCH ANGLE=(2,0,20) SPEED=(100,0,-40)
/**
* @brief  ��������
* @details  ����Dbus��CAN�������ݲ����Ƶ��ת��
* @param  ����������
* @retval  NULL
*/
void Control_Task(void* pvParameters)
{
	/*��������*/
	BaseType_t Dbus_Re = pdPASS;	/*Dbus��ֵ�ź�����ȡ����*/
	BaseType_t CAN_Re = pdPASS;		/*CAN��ֵ�ź�����ȡ����*/
	
	Motor_Pitch_Data.TargetPosition = 2700;	//�����ã���Ĭ���趨�ڽӽ�ˮƽ��λ��
	Motor_Yaw_Data.TargetPosition = 3350;	//�����ã����趨��ԭλ��
	
	PID_Init(&PID_Cloud_Pitch ,
	&Motor_Pitch_Data.TargetPosition ,
	&Motor_Pitch_Data.RealPosition ,	
	&Motor_Pitch_Data.TargetSpeed ,
	0 , 0 , 0 , 
	2000 , 30000);
	
	PID_Init(&PID_Cloud_Pitch_Speed , 
	& Motor_Pitch_Data.TargetSpeed ,
	& Motor_Pitch_Data.RealSpeed , 
	& Motor_Pitch_Data.TargetCurrent , 
	0 , 0 , 0 ,
	2000, 30000);
	
	PID_Init(&PID_Cloud_Yaw , 
	&Motor_Yaw_Data.TargetPosition , 
	&Motor_Yaw_Data.RealPosition , 
	&Motor_Yaw_Data.TargetSpeed , 
	0 , 0 , 0 ,
	2000 , 30000);
	
	PID_Init(&PID_Cloud_Yaw_Speed , 
	&Motor_Yaw_Data.TargetSpeed , 
	&Motor_Yaw_Data.RealSpeed	,
	&Motor_Yaw_Data.TargetCurrent	,
	0 , 0 , 0 ,
	2000 , 30000);
	
	while(1)
	{
		Dbus_Re = xSemaphoreTake(Dbus_Update_Handle,0);
//		CAN_Re = xSemaphoreTake(CAN_Update_Handle,0);
		if (Dbus_Re == pdTRUE) /*��ȡ����ֵ�ź�������ʼ����Dbus����*/
		{
			bsp_Dbus_Analysis(); /*Dbus���ݽ���*/
		}
		Motor_PID_Loop_Section(&PID_Cloud_Pitch);
		Motor_PID_Loop_Section(&PID_Cloud_Pitch_Speed);
		Motor_PID_Loop_Section(&PID_Cloud_Yaw);
		Motor_PID_Loop_Section(&PID_Cloud_Yaw_Speed);
		
//		CAN1_TxData16[0] = Motor_Yaw_Data.TargetCurrent;
//		CAN1_TxData16[1] = Motor_Pitch_Data.TargetCurrent;
//		
//		can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);
		
//		if (CAN_Re == pdTRUE)		/*��ȡ����ֵ�ź�������ʼ����CAN����*/
//		{
//			bsp_CAN_Analysis();	/*CAN���ݽ���*/
//		}
//		MotorRpm_Control(bsp_dbus_Data.CH_0 * 10); /*���Ƶ��ת��*/
		//MotorLoc_Control(5000);
//		bsp_CAN_Sendmessage(); /*CAN���߿������ݷ���*/
//		Jscope_show(M3508_Data.Rpm); /*Jscope��ʾ��������*/
		//Jscope_show(M3508_Data.Loc);
		vTaskDelay(1);
	}
}
int tempVx = 0;
int tempVy = 0;
int tempOmega = 0;
PID_t PID_M1,PID_M2, PID_M3, PID_M4;
/**
* @brief  ���̲����ÿ��Ƴ���
* @details  

									�� X+

						M1----------M2
						|		 /|\		|
						|		/ | \		|
			��Y+		|			|			|
						|			|			|
						|						|
						M4----------M3	(����ͼ)

						M1 CAN2 0X201 
						M2 CAN2 0X202 
						M3 CAN2 0X203 
						M4 CAN2 0X204 
* @param[in]  
* @retval  
*/

void task_test_control(void* param)
{
	PID_Init( &PID_M1, 
	&Motor_1_Data.TargetSpeed, 
	&Motor_1_Data.RealSpeed, 
	&Motor_1_Data.TargetCurrent ,
	10 , 0 , 0 ,2000 , 15000 );
	PID_Init( &PID_M2, 
	&Motor_2_Data.TargetSpeed, 
	&Motor_2_Data.RealSpeed, 
	&Motor_2_Data.TargetCurrent ,
	10 , 0 , 0 ,2000 , 15000 );
	PID_Init( &PID_M3, 
	&Motor_3_Data.TargetSpeed, 
	&Motor_3_Data.RealSpeed, 
	&Motor_3_Data.TargetCurrent ,
	10 , 0 , 0 ,2000 , 15000 );
	PID_Init( &PID_M4, 
	&Motor_4_Data.TargetSpeed, 
	&Motor_4_Data.RealSpeed, 
	&Motor_4_Data.TargetCurrent ,
	10 , 0 , 0 ,2000 , 15000 );
	
	while(1)
	{
		if(Chassis_Control_Data.cmd == 0 )
		{
			Chassis_Stop();
		}
		else
		{
			
			Chassis_Run( Chassis_Control_Data.speed_x*10000/660 , Chassis_Control_Data.speed_y*10000/660 , Chassis_Control_Data.speed_z*10000/660 );
//			Chassis_Run( tempVx , tempVy , tempOmega );
			
			Motor_PID_Loop_Section(&PID_M1);
			Motor_PID_Loop_Section(&PID_M2);
			Motor_PID_Loop_Section(&PID_M3);
			Motor_PID_Loop_Section(&PID_M4);
			
			
			CAN2_TxData16[0] = Motor_1_Data.TargetCurrent;
			CAN2_TxData16[1] = Motor_2_Data.TargetCurrent;
			CAN2_TxData16[2] = Motor_3_Data.TargetCurrent;
			CAN2_TxData16[3] = Motor_4_Data.TargetCurrent;
			can_send_msg(&hcan2, 0x200, CAN2_TxData16);

			osDelay(1);
		}
	}
}
