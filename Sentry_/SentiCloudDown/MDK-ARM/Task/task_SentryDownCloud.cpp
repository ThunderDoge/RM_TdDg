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

TaskHandle_t task_Main_Handle,task_CommuRoutine_Handle,task_CheckDevice_Handle;
uint32_t mark1, mark2;


static uint8_t is_cloud_inited=0;   // �ڱ���ʼ����־λ

/**
  * @brief  ��̨�ܳ�ʼ������
  * @details  
  * @param[in]
  * @retval  
  */
void DownCloud_Init(void)
{
    bsp_spi_Icm20602Init(); //������Icm20602��ʼ������SPI��
	
    app_imu_Init();         //���������ݴ���app_imu��ʼ��
	
#ifndef	MIGRATE_F407ZG
    bsp_can_Init();  //CAN���߳�ʼ������
#endif //MIGRATE_F407ZG

    manager::CANSelect(&hcan1, &hcan2); //��can������ʼ����ѡCAN��
	
	// ���߼���ʼ��
	app_sentry_CheckDevice_Init();
	// �豸��ӵ��豸�б�
    app_sentry_CheckDevice_AddToArray(&UpCloudRightFric_CheckDevice);
    app_sentry_CheckDevice_AddToArray(&UpCloudLeftFric_CheckDevice);
    app_sentry_CheckDevice_AddToArray(&UpCloudYawMotor_CheckDevice);
    app_sentry_CheckDevice_AddToArray(&UpCloudYawMotor_CheckDevice);
    app_sentry_CheckDevice_AddToArray(&UpCloudFeedMotor_CheckDevice);
	
	// ���߼��ṹ�� ����
	app_sentry_CheckDevice_AddToArray(&Dbus_CheckDevice);
    app_sentry_CheckDevice_AddToArray(&IMU_CheckDevice);

	// ��ʼ����ʶ����
    is_cloud_inited = 1;
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

    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        app_imu_So3thread();    //��ȡ����������
		DownCloudEntity.Handle();	//��̨���ݴ������������������app_imu_So3thread֮����á�
        ModeSelect();           //�ֱ�ң��ģʽ��ʼ��
        manager::CANSend();     //ͳһ��CAN�������
        vTaskDelayUntil(&LastTick, 1 / portTICK_PERIOD_MS );  //��ʱ1ms
		
		
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark1 = uxTaskGetStackHighWaterMark(task_Main_Handle);  //ռ�ö�ջˮλ�ߡ�������DEBUG
		#endif
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
		// CloudVisonTxRoutine();  //��̨�Ӿ����ڷ���
		UpCloudCanCommuRoutine(); //����̨CAN����
		vTaskDelayUntil(&LastTick,2 / portTICK_PERIOD_MS);   //��ʱ2ms
		
		#ifdef INCLUDE_uxTaskGetStackHighWaterMark
		mark2 = uxTaskGetStackHighWaterMark(task_CommuRoutine_Handle);  //ռ�ö�ջˮλ�ߡ�������DEBUG
		#endif
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
    if(!is_cloud_inited){   // ȷ�ϳ�ʼ��
        DownCloud_Init();
    }
    xTaskCreate((TaskFunction_t)	task_Main,		//�������
				(char*)				"task_Main",	//������
				(uint16_t)			512,			//��ջ���
				(void*)				NULL,			//�����б�
				(UBaseType_t)		4,				//���ȼ�
				(TaskHandle_t*)		&task_Main_Handle	);
				
	xTaskCreate((TaskFunction_t)	task_CommuRoutine,
				(char*)				"task_CommuRoutine",
				(uint16_t)			512,
				(void*)				NULL,
				(UBaseType_t)		4,
				(TaskHandle_t*)		&task_CommuRoutine_Handle);
				
	// xTaskCreate((TaskFunction_t)	task_CheckDevice,
	// 			(char*)				"task_CheckDevice",
	// 			(uint16_t)			512,
	// 			(void*)				NULL,
	// 			(UBaseType_t)		4,
	// 			(TaskHandle_t*)		&task_CheckDevice_Handle);
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
