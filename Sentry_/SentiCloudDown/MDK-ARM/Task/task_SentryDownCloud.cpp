/**
 * @file task_SentryDownCloud.cpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Tasks of SentryCloud
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "task_SentryDownCloud.hpp"

TaskHandle_t task_Main_Handle,task_CommuRoutine_Handle,task_Check_Handle;
uint32_t mark1, mark2;
uint32_t mark3;


static uint8_t is_cloud_inited=0;   // 哨兵初始化标志位

/**
  * @brief  云台总初始化函数
  * @details  
  * @param[in]
  * @retval  
  */
void DownCloud_Init(void)
{
    bsp_spi_Icm20602Init(); //陀螺仪Icm20602初始化，在SPI上
	
    app_imu_Init();         //陀螺仪数据处理app_imu初始化
	
#ifndef	MIGRATE_F407ZG
    bsp_can_Init();  //CAN总线初始化函数
#endif //MIGRATE_F407ZG

    manager::CANSelect(&hcan1, &hcan2); //大疆can电机库初始化（选CAN）
	

	// 初始化标识变量
    is_cloud_inited = 1;
}
/**
  * @brief  主任务
  * @details  
  * @param[in]  
  * @retval  
  */
    void
    task_Main(void *param)
{

    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        app_imu_So3thread();    //获取陀螺仪数据
		DownCloudEntity.Handle();	//云台数据处理，电机动作。必须在app_imu_So3thread之后调用。
        ModeSelect();           //手柄遥控模式初始化
        manager::CANSend();     //统一的CAN电机控制
        vTaskDelayUntil(&LastTick, 1 / portTICK_PERIOD_MS );  //延时1ms
		
		
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark1 = uxTaskGetStackHighWaterMark(task_Main_Handle);  //占用堆栈水位线。备用于DEBUG
		#endif
    }
}
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
		// CloudVisonTxRoutine();  //云台视觉串口发送
		DownCloudCanCommuRoutine(); //上云台CAN发送
		vTaskDelayUntil(&LastTick,2 / portTICK_PERIOD_MS);   //延时2ms
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark2 = uxTaskGetStackHighWaterMark(task_CommuRoutine_Handle);  //占用堆栈水位线。备用于DEBUG
		#endif
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
	app_check_EnableDevice(id_UpCloudConnect,500);
    app_check_SignDeviceTickTo(id_UpCloudConnect,&CanRx.CanUpdateTime[tUpCloud_Info]);
    app_check_EnableDevice(id_ChassisConnect,500);
    app_check_SignDeviceTickTo(id_ChassisConnect,&CanRx.CanUpdateTime[tChassis_Info]);
    // 电机设定
    app_check_EnableDevice(id_DownCloudPitchMotor,50);
    app_check_SignDeviceTickTo(id_DownCloudPitchMotor,&DownCloudEntity.PitchMotor.LastUpdateTime);
    app_check_EnableDevice(id_DownCloudYawMotor,50);
    app_check_SignDeviceTickTo(id_DownCloudYawMotor,&DownCloudEntity.YawMotor.LastUpdateTime);
    app_check_EnableDevice(id_DownCloudLeftFric,50);
    app_check_SignDeviceTickTo(id_DownCloudLeftFric,&DownCloudEntity.FricLeftMotor.LastUpdateTime);
    app_check_EnableDevice(id_DownCloudRightFric,50);
    app_check_SignDeviceTickTo(id_DownCloudRightFric,&DownCloudEntity.FricRightMotor.LastUpdateTime);
    app_check_EnableDevice(id_DownCloudFeedMotor,50);
    app_check_SignDeviceTickTo(id_DownCloudFeedMotor,&DownCloudEntity.Feed2nd.LastUpdateTime);

    
    while (1)
    {
        app_check_RefreshList();    // 刷新离线列表
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark3 = uxTaskGetStackHighWaterMark(task_Check_Handle);  //占用堆栈水位线。备用于DEBUG
		#endif

        vTaskDelay(10/portTICK_PERIOD_MS);  
    }
    
}




/**
  * @brief  任务启动器
  * @details  
  * @param[in]  
  * @retval  
  */
void TaskStarter(void)
{
    if(!is_cloud_inited){   // 确认初始化
        DownCloud_Init();
    }
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
				(UBaseType_t)		4,
				(TaskHandle_t*)		&task_CommuRoutine_Handle);
				
	xTaskCreate((TaskFunction_t)	task_Check,
				(char*)				"task_Check",
				(uint16_t)			1024,
				(void*)				NULL,
				(UBaseType_t)		4,
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