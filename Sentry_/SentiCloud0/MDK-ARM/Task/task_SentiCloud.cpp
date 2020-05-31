/**
 * @file      task_SentiCloud.cpp
 * @brief     哨兵云台RTOS任务
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-15
 * @version   v1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 * 
 * v1.0 2020-4-15   标准版本发布. 已添加离线检测
 */
#include "task_SentiCloud.hpp"
#include "bsp_gy53l1.h"




/**
  * @brief  云台总初始化函数
  * @details  
  * @param[in]
  * @retval  
  */
void Cloud_Init(void)
{
    bsp_spi_Icm20602Init(); //陀螺仪Icm20602初始化，在SPI上
    app_imu_Init();         //陀螺仪数据处理app_imu初始化
	#ifndef	MIGRATE_F407ZG	// 在个人的开发板上测试时的宏定义
    bsp_can_Init();  //CAN总线初始化函数
#endif //MIGRATE_F407ZG
    bsp_dbus_Init(); //DBUS初始化
	Dbus_CHx_StaticOffset[1] = -4;	//这是遥控器摇杆静态误差。跟特定遥控器相关，换遥控器请更改此值。
	app_vision_Init();              //视觉串口接收初始化
    manager::CANSelect(&hcan1, &hcan2); //大疆can电机库初始化（选CAN）
	HAL_Delay(100);
	Dbus_CHx_StaticOffset[0] = -bsp_dbus_Data.CH_0;
	Dbus_CHx_StaticOffset[1] = -bsp_dbus_Data.CH_1;
	Dbus_CHx_StaticOffset[2] = -bsp_dbus_Data.CH_2;
	Dbus_CHx_StaticOffset[3] = -bsp_dbus_Data.CH_3;
}
/**
  * @brief  主任务
  * @details  
  * @param[in]  
  * @retval  
  */
void task_Main(void *param)
{

    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        app_imu_So3thread();    //获取陀螺仪数据
        ModeSelect();           //手柄遥控模式初始化
		CloudEntity.Handle();	//云台数据处理，电机动作。必须在app_imu_So3thread之后调用。
        manager::CANSend();     //统一的CAN电机控制
        vTaskDelayUntil(&LastTick, 1 / portTICK_PERIOD_MS );  //延时1ms
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark1 = uxTaskGetStackHighWaterMark(task_Main_Handle);  //占用堆栈水位线。备用于DEBUG
		#endif
    }
}
char s_u5[]="superior_uart5_tx_test\r\n";
/**
  * @brief  视觉串口周期性发送任务+CAN通信周期性发送任务
  * @details  因为执行周期不同所以和主任务分开。100HZ运行。
  * @param[in]  
  * @retval  
  */
void task_CommuRoutine(void *param)
{
    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
		CloudVisonTxRoutine();  //云台视觉串口发送
		UpCloudCanCommuRoutine(); //上云台CAN发送
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark2 = uxTaskGetStackHighWaterMark(task_CommuRoutine_Handle);  //占用堆栈水位线。备用于DEBUG
		#endif
//		vTaskDelayUntil(&LastTick,10 / portTICK_PERIOD_MS);
		vTaskDelay( 2 / portTICK_PERIOD_MS );
	}
}

/**
 * @brief 设备离线检测任务
 * 
 * @param     param 
 */
void task_Check(void* param)
{
	app_check_Init();

    // 与云台的连接
	app_check_EnableDevice(id_DownCloudConnect,500);
    app_check_SignDeviceTickTo(id_DownCloudConnect,&CanRx.CanUpdateTime[tDownCloud_Info]);
    app_check_EnableDevice(id_ChassisConnect,500);
    app_check_SignDeviceTickTo(id_ChassisConnect,&CanRx.CanUpdateTime[tChassis_Info]);
    // DBUS
    app_check_EnableDevice(id_Dbus,50);
    app_check_SignDeviceTickTo(id_Dbus,&bsp_dbus_Data.UpdateTick);
    // 电机设定
    app_check_EnableDevice(id_UpCloudPitchMotor,50);
    app_check_SignDeviceTickTo(id_UpCloudPitchMotor,&CloudEntity.PitchMotor.LastUpdateTime);
    app_check_EnableDevice(id_UpCloudYawMotor,50);
    app_check_SignDeviceTickTo(id_UpCloudYawMotor,&CloudEntity.YawMotor.LastUpdateTime);
    app_check_EnableDevice(id_UpCloudLeftFric,50);
    app_check_SignDeviceTickTo(id_UpCloudLeftFric,&CloudEntity.FricLeftMotor.LastUpdateTime);
    app_check_EnableDevice(id_UpCloudRightFric,50);
    app_check_SignDeviceTickTo(id_UpCloudRightFric,&CloudEntity.FricRightMotor.LastUpdateTime);
    app_check_EnableDevice(id_UpCloudFeedMotor,50);
    app_check_SignDeviceTickTo(id_UpCloudFeedMotor,&CloudEntity.Feed2nd.LastUpdateTime);

    
    while (1)
    {
        app_check_RefreshList();    // 刷新离线列表
        vTaskDelay(10/portTICK_PERIOD_MS);  
    }
    
}





TaskHandle_t task_Main_Handle,task_CommuRoutine_Handle;
uint32_t mark1, mark2;
TaskHandle_t task_Check_Handle;
uint32_t mark3;

/**
  * @brief  任务启动器
  * @details  
  * @param[in]  
  * @retval  
  */
void TaskStarter(void)
{
    xTaskCreate((TaskFunction_t)	task_Main,		//任务代码
				(char*)				"task_Main",	//任务名
				(uint16_t)			1024,			//堆栈深度
				(void*)				NULL,			//参数列表
				(UBaseType_t)		4,				//优先级
				(TaskHandle_t*)		&task_Main_Handle	);
				
	xTaskCreate((TaskFunction_t)	task_CommuRoutine,
				(char*)				"task_CommuRoutine",
				(uint16_t)			1024,
				(void*)				NULL,
				(UBaseType_t)		2,
				(TaskHandle_t*)		&task_CommuRoutine_Handle);

	xTaskCreate((TaskFunction_t)	task_Check,
				(char*)				"task_Check",
				(uint16_t)			1024,
				(void*)				NULL,
				(UBaseType_t)		3,
				(TaskHandle_t*)		&task_Check_Handle);
							
}
//CAN线测试
// int16_t test_data[4];
// int16_t can_received;
// int16_t last_recv;
// int16_t recv_cnt;
// float recv_rate;
// int delau_CCNNTT = 500;
// void RecvFromCan(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef* RxHead,uint8_t* Data)
// {
// 	if(_hcan == &hcan2 )
// 		if(RxHead->StdId == 0x1F0)
// 			can_received = Data[0]<<8 | Data[1];
// 	if(can_received>last_recv)
// 	{
// 		recv_cnt++;
// 		last_recv = can_received;
// 	}
// 	else if(can_received < last_recv)
// 	{
// 		last_recv = can_received;
// 		recv_rate = (float)recv_cnt / 30000.0f;
// 		recv_cnt=0;
// 	}
// }
// void task_can_test(void*param)
// {
// 	int i,k;
// 	while(1)
// 	{
// 		for(i=0;i<30000;i++)
// 		{
// 			test_data[0] = i;
// 			bsp_can_Sendmessage(&hcan1,0x1F0,test_data);
// 			for(int j=0;j<delau_CCNNTT;j++){k=j;}
// //			HAL_Delay(1);
// 		}
// 	}
// }
