/**
  * @brief     云台视觉串口通信管理
  * @details
  * @author   ThunderDoge
  * @date
  * @version
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
  */
#include "SentryCloudVision.hpp"

void CloudVisionRoutine(void)
{
	bsp_vision_Send_Data.Pitch  = Self.RealPitch;
	bsp_vision_Send_Data.Yaw    = Self.RealYaw;
//		bsp_vision_Send_Data.Pitch  = 10.0;
//	bsp_vision_Send_Data.Yaw    = 25.5;

	bsp_vision_Send_Data.Ready_flag = 1;
	bsp_vision_SendData((uint8_t)CMD_GET_MCU_STATE);
}




