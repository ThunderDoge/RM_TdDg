/** 
 * @brief    哨兵RTOS任务
 * @details  
 * @author   ThunderDoge
 * @date      
 * @version  v0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
 
 #include "task_sentCha.hpp"
void TaskStarter(void)
{
	RoboInit();
	xTaskCreate(task_Main,"task_Main",512,NULL,4,NULL);
	xTaskCreate(task_Commu,"task_Commu",512,NULL,4,NULL);
}
/**
  * @brief  统一初始化程序
  * @details  包含：DBUS，CAN，
  */
void RoboInit()
{
	taskENTER_CRITICAL();
	bsp_spi_Icm20602Init();
	app_imu_Init();
	taskEXIT_CRITICAL();
//	bsp_dbus_Init();
	bsp_can_Init();
	bsp_Current_Init();
	bsp_encoder_Init(2048);
	bsp_ADC_Sensor_Init();
	
	manager::CANSelect(&hcan1,&hcan2);
}
/**
 * @brief  主任务
 * @details  
 */
void task_Main(void* param)
{
	bsp_encoder_SetValue(0);
	TickType_t LastTick = xTaskGetTickCount();
	while (1)
	{
		bsp_Current_Read();
		bsp_encoder_Handle();
		app_imu_So3thread();
        Self.Handle();
		ModeSelect();
//		manager::CANSend();	
		vTaskDelayUntil(&LastTick,1);
	}
}

void ChassisCanCommuRoutine(void);	//声明一下，避免麻烦的文件包含
void task_Commu(void* param)
{
	TickType_t LastTick = xTaskGetTickCount();

	while(1)
	{
		ChassisCanCommuRoutine();
		vTaskDelayUntil(&LastTick,5);

	}
}

//Aborted code following

//电机控制用PID和电机类
// Motor_t DJI_3508(8192,19);
// pid dwPid_in(0.25,0,0,1000,16000,100,300);
// pid dwPid_out(0,0,0,1000,1000,10,200);
// softmotor DriveWheel(1,0x201,&DJI_3508,&dwPid_in,&dwPid_out);
// float set_speed;
// uint8_t mode;
// /**
//   * @brief  遥控器测试模式
//   * @details  
//   * @param[in]  
//   * @retval  
//   */
// void RemoteTestMode()
// {
// 	set_speed = bsp_dbus_Data.CH_0 * 10000.0f / 660.0f;
// 	DriveWheel.Speed_Set(set_speed);
// }
// /**
//   * @brief  移交小主机控制模式
//   * @details  
//   * @param[in]  
//   * @retval  
//   */
// void NUCTestMode()
// {

// }
// void MainModeSelect(void)
// {
// 	mode = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;
// 	switch (mode) {
// 	case 32:	//左中右下
// 		RemoteTestMode();
// 	break;
// 	case 12:	//左上右下
// 		NUCTestMode();
// 	break;
// 	}
// }


 
