/** 
 * @brief    �ڱ�RTOS����
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
}
/**
  * @brief  ͳһ��ʼ������
  * @details  ������DBUS��CAN��
  */
void RoboInit()
{
	taskENTER_CRITICAL();
	bsp_spi_Icm20602Init();
	app_imu_Init();
	taskEXIT_CRITICAL();
	bsp_dbus_Init();
	bsp_can_Init();
	
	manager::CANSelect(&hcan1,&hcan2);
}

//���������PID�͵����
// Motor_t DJI_3508(8192,19);
// pid dwPid_in(0.25,0,0,1000,16000,100,300);
// pid dwPid_out(0,0,0,1000,1000,10,200);
// softmotor DriveWheel(1,0x201,&DJI_3508,&dwPid_in,&dwPid_out);
// float set_speed;
// uint8_t mode;
// /**
//   * @brief  ң��������ģʽ
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
//   * @brief  �ƽ�С��������ģʽ
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
// 	case 32:	//��������
// 		RemoteTestMode();
// 	break;
// 	case 12:	//��������
// 		NUCTestMode();
// 	break;
// 	}
// }


 
/**
 * @brief  ������
 * @details  
 */
void task_Main(void* param)
{
	while (1)
	{
		app_imu_So3thread();
        Self.Handle();
		ModeSelect();

		manager::CANSend();	
	}
	
}
