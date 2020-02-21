/**
 * @file SentryCloudVision.cpp
  * @brief     云台视觉串口通信
  * @details  提供一个云台与他的小主机妈妈进行UART通讯的所有函数。包含接受到的信息：处理-储存，将要发送的信息：打包，发送。
  * 仅仅依赖于bsp_vision。注意bsp_vision。这意味着她不能够主动调用其他的组件的函数。所以你将看到很多的**回调函数**的写法
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



