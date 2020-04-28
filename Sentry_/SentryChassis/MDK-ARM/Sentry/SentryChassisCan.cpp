/**
  * @file  SentryChassisCan.cpp
  * @brief      
  * @details    
  * @author     ThunderDoge
  * @date       2020-2-23
  * @version    0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryChassisCan.hpp"

void ChassisCanFilterConfig()
{
    
}
/**
* @brief  CAN接收中断
* @details  重新定义接收中断，会自动在CAN中断中调用，不需要手动添加,使用的时候自行在此函数中替换解析函数
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	static CAN_RxHeaderTypeDef bsp_can_Rx;
	uint8_t CAN_RxData[8];
	
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //判断中断产生
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//获取CAN报文
		motor::CANUpdate(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//电机信息更新
		CanRxCpltCallBack_ChassisCommuUpdata(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//板间CAN通信信息更新
	}
}



