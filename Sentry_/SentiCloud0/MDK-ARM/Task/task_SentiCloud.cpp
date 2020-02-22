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
  * @brief  ��̨�ܳ�ʼ������
  * @details  
  * @param[in]
  * @retval  
  */
void Cloud_Init(void)
{
    bsp_spi_Icm20602Init(); //������Icm20602��ʼ������SPI��
    app_imu_Init();         //���������ݴ���app_imu��ʼ��
#ifndef	MIGRATE_F407ZG
    bsp_can_Init();  //CAN���߳�ʼ������
#endif //MIGRATE_F407ZG
    bsp_dbus_Init(); //DBUS��ʼ��
	Dbus_CHx_StaticOffset[1] = -4;	//����ң����ҡ�˾�̬�����ض�ң������أ���ң��������Ĵ�ֵ��
	bsp_vision_Init();              //�Ӿ����ڽ��ճ�ʼ��
    manager::CANSelect(&hcan1, &hcan2); //��can������ʼ����ѡCAN��
}
/**
  * @brief  ������
  * @details  
  * @param[in]  
  * @retval  
  */
    void
    task_Main(void *param)
{
//	while(Self.PitchMotor.RealAngle == 0){;}
//    app_imu_data.integral.Pitch = Self.PitchMotor.RealAngle; //ע�⸺�š�
//	while(Self.YawMotor.RealAngle == 0){;}
//	app_imu_data.integral.Yaw = -Self.YawMotor.RealAngle;

    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        app_imu_So3thread();    //��ȡ����������
		CloudEntity.Handle();	//��̨���ݴ������������������app_imu_So3thread֮����á�
        ModeSelect();           //�ֱ�ң��ģʽ��ʼ��
        manager::CANSend();     //ͳһ��CAN�������
        vTaskDelayUntil(&LastTick, 1);  //��ʱ1Tick(Ĭ��Ϊ1ms)
		mark1 = uxTaskGetStackHighWaterMark(task_Main_Handle);  //ռ�ö�ջˮλ�ߡ�������DEBUG
    }
}
/**
  * @brief  �Ӿ����������Է�������+CANͨ�������Է�������
  * @details  ��Ϊִ�����ڲ�ͬ���Ժ�������ֿ���100HZ���С�
  * @param[in]  
  * @retval  
  */
void task_CommuRoutine(void *param)
{
    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
		CloudVisonTxRoutine();  //��̨�Ӿ����ڷ���
		UpCloudCanCommuRoutine(); //����̨CAN����
		mark2 = uxTaskGetStackHighWaterMark(task_CommuRoutine_Handle);  //ռ�ö�ջˮλ�ߡ�������DEBUG
		vTaskDelayUntil(&LastTick,2);   //��ʱ2Tick
    }
}
/**
  * @brief  ����������
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
//CAN�߲���
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
