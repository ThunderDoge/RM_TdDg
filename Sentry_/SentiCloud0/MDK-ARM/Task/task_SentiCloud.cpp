/**
 * @file task_SentiCloud.cpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Tasks of SentryCloud
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "task_SentiCloud.hpp"

TaskHandle_t task_Main_Handle,task_CommuRoutine_Handle;
uint32_t mark1, mark2;


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

    bsp_can_Init();  //CAN总线初始化函数
    bsp_dbus_Init(); //DBUS初始化
	Dbus_CHx_StaticOffset[1] = -4;	//这是遥控器摇杆静态误差。跟特定遥控器相关，换遥控器请更改此值。
	bsp_vision_Init();              //视觉串口接收初始化
    manager::CANSelect(&hcan1, &hcan2); //大疆can电机库初始化（选CAN）
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
//	while(Self.PitchMotor.RealAngle == 0){;}
//    app_imu_data.integral.Pitch = Self.PitchMotor.RealAngle; //注意负号。
//	while(Self.YawMotor.RealAngle == 0){;}
//	app_imu_data.integral.Yaw = -Self.YawMotor.RealAngle;

    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        app_imu_So3thread();    //获取陀螺仪数据
		CloudEntity.Handle();	//云台数据处理，电机动作。必须在app_imu_So3thread之后调用。
        ModeSelect();           //手柄遥控模式初始化
        manager::CANSend();     //统一的CAN电机控制
        vTaskDelayUntil(&LastTick, 1);  //延时1Tick(默认为1ms)
		mark1 = uxTaskGetStackHighWaterMark(task_Main_Handle);  //占用堆栈水位线。备用于DEBUG
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
		CloudVisonTxRoutine();  //云台视觉串口发送
		UpCloudCanCommuRoutine(); //上云台CAN发送
		mark2 = uxTaskGetStackHighWaterMark(task_CommuRoutine_Handle);  //占用堆栈水位线。备用于DEBUG
		vTaskDelayUntil(&LastTick,2);   //延时2Tick
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
    Cloud_Init();
    xTaskCreate(task_Main, "task_Main", 512, NULL, 4, &task_Main_Handle);   //512Byte, Priority=4
	xTaskCreate(task_CommuRoutine,"task_CommuRoutine",512,NULL,4,&task_CommuRoutine_Handle);    //512Byte, Priority=4
//	xTaskCreate(task_SentryTroubleShooter,"task_SentryTroubleShooter",4096,NULL,4,NULL);    //
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
