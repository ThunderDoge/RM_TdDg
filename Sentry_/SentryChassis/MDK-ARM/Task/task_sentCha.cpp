/** 
 * @brief    哨兵RTOS任务
 * @details  
 * @author   ThunderDoge
 * @date      
 * @version  v0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */

#include "task_sentCha.hpp"

 /**
  * @brief 统一的任务启动器
  */
void TaskStarter(void)
{
	xTaskCreate(task_Main,"task_Main",1024,NULL,4,NULL);
	xTaskCreate(task_Commu,"task_Commu",1024,NULL,4,NULL);
	xTaskCreate(task_OfflineCheck,"task_OfflineCheck",1024,NULL,4,NULL);
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
#ifndef	MIGRATE_F407ZG
    bsp_can_Init();  //CAN总线初始化函数
#endif //MIGRATE_F407ZG
	bsp_Current_Init();
	bsp_encoder_Init(2048);
	bsp_ADC_Sensor_Init();
	
	manager::CANSelect(&hcan1,&hcan2);
	
	ChassisCanInit();
}
/**
 * @brief  主任务,1KHZ
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
        ChassisEntity.Handle();
		ModeSelect();
//		manager::CANSend();	
		vTaskDelayUntil(&LastTick,1/portTICK_PERIOD_MS);
	}
}

/**
 * @brief 板间通讯用任务，200HZ
 * 
 * @param     param 
 */
void task_Commu(void* param)
{
	TickType_t LastTick = xTaskGetTickCount();

	while(1)
	{
		ChassisCanCommuRoutine();       // 板间CAN通讯发送
		vTaskDelayUntil(&LastTick,5/portTICK_PERIOD_MS);   
	}
}


uint32_t test_tick=0;
int update_test = 0;
int update_report=0;
void rep_ol(void){
	update_report = 1;
}
void rep_ofl(void){
	update_report=0;
}

/**
 * @brief   离线检测任务
 * 
 * @param     param 
 */
void task_OfflineCheck(void *param)
{
	// 设定离线检测
	app_check_Init();

	app_check_EnableDevice(id_UpCloudConnect,500);
    app_check_SignDeviceTickTo(id_UpCloudConnect,&CanRx.CanUpdateTime[tUpCloud_Info]);
	
    app_check_EnableDevice(id_DownCloudConnect,500);
    app_check_SignDeviceTickTo(id_DownCloudConnect,&CanRx.CanUpdateTime[tDownCloud_Info]);

	app_check_EnableDevice(id_ChassisDriveMotor,100);
	app_check_SignDeviceTickTo(id_ChassisDriveMotor,&ChassisEntity.DriveWheel.LastUpdateTime);

    app_check_EnableDevice(id_ChassisImu,100);
	app_check_SignDeviceTickTo(id_ChassisImu,&tImuLastTick);
	
    app_check_EnableDevice(id_ChassisRailEncoder,100);
	app_check_SignDeviceTickTo(id_ChassisRailEncoder,&bsp_encoder_UpdateTime);
	
    app_check_EnableDevice(id_ChassisLazerRangingLeft,150);
	app_check_SignDeviceTickTo(id_ChassisLazerRangingLeft,&ChassisEntity.RangingLeft.data.update_time);
    app_check_EnableDevice(id_ChassisLazerRangingRight,150);
	app_check_SignDeviceTickTo(id_ChassisLazerRangingRight,&ChassisEntity.RangingRight.data.update_time);

	app_check_EnableDevice(id_Test,1000);
	app_check_SignDeviceTickTo(id_Test,&test_tick);
	app_check_SignOnlineCallback(id_Test,rep_ol);
	app_check_SignOfflineCallback(id_Test,rep_ofl);
	while(1)
	{
		if(update_test)
			test_tick = HAL_GetTick();
		
		app_check_RefreshList();
		vTaskDelay(10/portTICK_PERIOD_MS);
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


 
