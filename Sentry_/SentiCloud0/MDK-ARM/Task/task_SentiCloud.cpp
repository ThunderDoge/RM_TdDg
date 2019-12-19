/**
  * @file:
  * @brief    RM2020 �ڱ�����
  * @details  
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "task_SentiCloud.hpp"

/**
  * @brief  ��̨�ܳ�ʼ������
  * @details  
  * @param[in]  void
  * @retval  
  */
void Cloud_Init(void)
{
    bsp_spi_Icm20602Init();
    app_imu_Init();

    bsp_can_Init();  //CAN���߳�ʼ������
    bsp_dbus_Init(); //DBUS��ʼ��
	Dbus_CHx_StaticOffset[1] = -4;	//����ң����ҡ�˾�̬�����ض�ң������أ���ң��������Ĵ�ֵ��
	bsp_vision_Init();
    manager::CANSelect(&hcan1, &hcan2);
    app_imu_data.integral.Roll = -Self.PitchMotor.RealAngle; //ע�⸺�š�
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
        app_imu_So3thread();
		Self.Handle();	//������������app_imu_So3thread֮����á�
        ModeSelect();
		//
//		testDt = HAL_GetTick() - testT;
//		testT = HAL_GetTick();
		//
        manager::CANSend();
        vTaskDelayUntil(&LastTick, 1);
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
		CloudVisonTxRoutine();
		CloudCanCommuRoutine();
		vTaskDelayUntil(&LastTick,5);
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
    xTaskCreate(task_Main, "task_Main", 512, NULL, 4, NULL);
	xTaskCreate(task_CommuRoutine,"task_CommuRoutine",512,NULL,4,NULL);
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
