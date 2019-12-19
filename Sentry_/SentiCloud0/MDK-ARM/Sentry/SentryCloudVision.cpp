/**
  * @brief     云台视觉串口通信管理
  * @details
  * @author   ThunderDoge
  * @date
  * @version
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
  */
#include "SentryCloudVision.hpp"
#define DEBUG
#ifndef DEBUG
#include "cmsis_os.h"
#endif

void CloudVisionRoutine(void)
{
	bsp_vision_Send_Data.Pitch  = Self.RealPitch;
	bsp_vision_Send_Data.Yaw    = Self.MechanicYaw;

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



