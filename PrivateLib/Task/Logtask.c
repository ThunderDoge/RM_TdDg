/** 
* @brief    检测系统运行状态的函数和任务
* @details  优先级较低的一个检测任务，初始化完成之前系统闪蓝灯，完成后闪绿灯，同时检测Dbus，CAN总线是否离线
* @author   郭俊辉
* @date      2019.10
* @version  1.0 
* @par Copyright (c):  RM2020电控
* @par 日志
*/

#include "Logtask.h"

//静态函数声明
static int8_t Dbus_Offline_Check(void);
static int8_t CAN_Offline_Check(uint8_t CAN_ID);
static int8_t MPU9250_Offline_Check(int8_t ID);

TaskHandle_t Log_Task_Handle = NULL;	/*系统状态任务句柄*/
SemaphoreHandle_t Dbus_Check_Handle = NULL; /*Dbus离线检测二值信号量*/
SemaphoreHandle_t CAN_Check_Handle = NULL;	/*CAN离线检测二值信号量*/
EventBits_t	Init_Event;

/**
* @brief  系统运行状态报告任务
* @details  显示系统运行状态
* @param  系统运行状态任务句柄
* @retval  NULL
*/
void Log_Task(void* pvParameters)
{
	/*系统运行标志任务*/
	EventBits_t Re_Init_Event;
	while(1)
	{
		Re_Init_Event = xEventGroupWaitBits(Inittask_Event_Handle,
																				Init_Event,
																				pdFALSE,
																				pdTRUE,
																				0);						/*获取初始化事件状态*/
		//printf("%d\n",Re_Init_Event);
		if((Re_Init_Event & (Init_Event)) == Init_Event)		/*初始化已经完成*/
		{
//			if (CAN_Offline_Check(201) == 0 || Dbus_Offline_Check()== 0)			/*CAN,Dbus总线离线检测*/
//			{
//				HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_TogglePin(RGB_R_GPIO_Port,RGB_R_Pin); /*状态灯指示*/
//				Music_Play(warningx);
//			}
//			else
			{
				HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);			/*所有设备正常在线*/
				HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(RGB_G_GPIO_Port,RGB_G_Pin); /*状态灯指示*/
				//printf("CH0:%d CH1:%d CH2:%d CH3:%d Dial:%d\n",Dbus_Data.CH_0,Dbus_Data.CH_1,Dbus_Data.CH_2,Dbus_Data.CH_3,Dbus_Data.Dial);
				//printf("X:%d Left:%d Right:%d\n",Dbus_Data.Mouse.X,Dbus_Data.Mouse.Leftkey,Dbus_Data.Mouse.Rightkey);
				//printf("ROLL:%4f PITCH:%4f YAW:%4f\n",imu.Roll,imu.Pitch,imu.Yaw);
			}
			Music_Handle();		/*蜂鸣器音乐状态更新*/
		}
		else																								/*初始化还没有完成*/
		{
			HAL_GPIO_TogglePin(RGB_B_GPIO_Port,RGB_B_Pin); /*状态灯指示*/
			HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
		}
		vTaskDelay(100);
	}
}

/**
* @brief  Dbus离线检测
* @details  检测Dbus信号是否在线
* @param  NULL
* @retval  0离线 1在线
*/
static int8_t Dbus_Offline_Check(void)
{
	/*Dbus信号离线检测*/
	static int16_t Offline_Cnt = 0;
	static BaseType_t Dbus_Check = pdPASS;
	Dbus_Check = xSemaphoreTake(Dbus_Check_Handle,0);
//	if (Dbus_Check == pdTRUE)		/*根据二值信号量更新数据检测离线*/
//	{
//		return 1;	/*Dbus在线*/
//	}
//	else
//	{
//		return 0;	/*Dbus离线*/
//	}
	if (Dbus_Check == pdTRUE)		/*根据二值信号量更新数据检测离线*/
	{
		Offline_Cnt = 0;
	}
	else
	{
		Offline_Cnt ++;
	}
	//printf("%d\n",Offline_Cnt);
	if (Offline_Cnt >= 5)			/*超时即判断离线，此处阈值0.5S*/
	{
		if(Offline_Cnt == 32767) /*防止变量溢出*/
		{
			Offline_Cnt = 5;
		}
		//printf("Dbus offline\n");
		return 0;				/*Dbus离线*/
	}
	else 
	{
		//printf("Dbus online\n");
		return 1;				/*Dbus在线*/
	}
}

/**
* @brief  CAN总线离线检测
* @details  检测CAN总线上指定ID号设备是否离线
* @param  设备ID号
* @retval  0离线 1在线
*/
static int8_t CAN_Offline_Check(uint8_t CAN_ID)
{
	/*CAN总线离线检测*/
	static int16_t Offline_Cnt = 0;
	static BaseType_t CAN_Check = pdPASS;
	CAN_Check = xSemaphoreTake(CAN_Check_Handle,0);
	if (CAN_Check == pdTRUE)		/*根据二值信号量更新数据检测离线*/
	{
		Offline_Cnt = 0;
	}
	else
	{
		Offline_Cnt ++;
	}
	//printf("%d\n",Offline_Cnt);
	if (Offline_Cnt >= 5)			/*超时即判断离线，此处阈值0.5S*/
	{
		if(Offline_Cnt == 32767) /*防止变量溢出*/
		{
			Offline_Cnt = 5;
		}
		//printf("Dbus offline\n");
		return 0;				/*CAN离线*/
	}
	else 
	{
		//printf("Dbus online\n");
		return 1;				/*CAN在线*/
	}
}

/**
* @brief  MPU9250在线检测
* @details  读取9250ID号
* @param  ID ID号
* @retval 1在线 0离线 
*/
static int8_t MPU9250_Offline_Check(int8_t ID)
{
	if(bsp_SPI_MPU9250_Readreg(0X75)!=0X71)
		return 0;				/*MPU9250离线*/
	if(bsp_SPI_MPU9250_Readreg(0X75)==0X71)
		return 1;			/*MPU9250在线*/
}
