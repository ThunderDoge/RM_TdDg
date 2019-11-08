/** 
* @brief    系统控制任务
* @details  解析CAN,Dbus数据，控制电机转动
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/
#include "Controltask.h"

TaskHandle_t Control_Task_Handle = NULL;	/*控制任务句柄*/
SemaphoreHandle_t Dbus_Update_Handle = NULL; /*Dbus二值化更新变量*/
SemaphoreHandle_t CAN_Update_Handle = NULL; /*CAN二值化更新变量*/
 
//PITCH ANGLE=(2,0,20) SPEED=(100,0,-40)
/**
* @brief  控制任务
* @details  更新Dbus和CAN总线数据并控制电机转动
* @param  控制任务句柄
* @retval  NULL
*/
void Control_Task(void* pvParameters)
{
	/*控制任务*/
	BaseType_t Dbus_Re = pdPASS;	/*Dbus二值信号量获取变量*/
	BaseType_t CAN_Re = pdPASS;		/*CAN二值信号量获取变量*/
	
	Motor_Pitch_Data.TargetPosition = 2700;	//测试用，先默认设定在接近水平的位置
	Motor_Yaw_Data.TargetPosition = 3350;	//测试用，先设定在原位；
	
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
		if (Dbus_Re == pdTRUE) /*获取到二值信号量，开始更新Dbus数据*/
		{
			bsp_Dbus_Analysis(); /*Dbus数据解析*/
		}
		Motor_PID_Loop_Section(&PID_Cloud_Pitch);
		Motor_PID_Loop_Section(&PID_Cloud_Pitch_Speed);
		Motor_PID_Loop_Section(&PID_Cloud_Yaw);
		Motor_PID_Loop_Section(&PID_Cloud_Yaw_Speed);
		
//		CAN1_TxData16[0] = Motor_Yaw_Data.TargetCurrent;
//		CAN1_TxData16[1] = Motor_Pitch_Data.TargetCurrent;
//		
//		can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);
		
//		if (CAN_Re == pdTRUE)		/*获取到二值信号量，开始更新CAN数据*/
//		{
//			bsp_CAN_Analysis();	/*CAN数据解析*/
//		}
//		MotorRpm_Control(bsp_dbus_Data.CH_0 * 10); /*控制电机转速*/
		//MotorLoc_Control(5000);
//		bsp_CAN_Sendmessage(); /*CAN总线控制数据发送*/
//		Jscope_show(M3508_Data.Rpm); /*Jscope显示变量更新*/
		//Jscope_show(M3508_Data.Loc);
		vTaskDelay(1);
	}
}
int tempVx = 0;
int tempVy = 0;
int tempOmega = 0;
PID_t PID_M1,PID_M2, PID_M3, PID_M4;
/**
* @brief  底盘测试用控制程序
* @details  

									↑ X+

						M1----------M2
						|		 /|\		|
						|		/ | \		|
			←Y+		|			|			|
						|			|			|
						|						|
						M4----------M3	(俯视图)

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
