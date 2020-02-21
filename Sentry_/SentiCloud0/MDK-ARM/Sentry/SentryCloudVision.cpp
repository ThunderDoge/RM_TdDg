/**
 * @file SentryCloudVision.cpp
  * @brief     ��̨�Ӿ�����ͨ��
  * @details  �ṩһ����̨������С�����������UARTͨѶ�����к������������ܵ�����Ϣ������-���棬��Ҫ���͵���Ϣ����������͡�
  * ����������bsp_vision��ע��bsp_vision������ζ�������ܹ�������������������ĺ����������㽫�����ܶ��**�ص�����**��д��
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudVision.hpp"
// #define DEBUG

void CloudVisionRoutine(void)
{
	bsp_vision_Send_Data.Pitch  = CloudEntity.RealPitch;
	bsp_vision_Send_Data.Yaw    = CloudEntity.MechanicYaw;

	bsp_vision_Send_Data.Ready_flag = 1;
	bsp_vision_SendData((uint8_t)CMD_GET_MCU_STATE);
	#ifndef DEBUG
	vTaskDelay(1);
    bsp_vision_Send_Data.pillar_flag = CloudCanRecv.pillar_close_flag;
    bsp_vision_Send_Data.Px = CloudCanRecv.ChassisLocation;
    bsp_vision_SendData((uint8_t)STA_CHASSIS);
	#endif
}
#undef DEBUG 



