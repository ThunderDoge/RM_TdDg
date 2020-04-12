//
/**
  * @file SentryCloudCan.cpp
  * @brief    哨兵云台CAN通信
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryDownCloudCan.hpp"
#include <cstring>



//全局CAN接收变量
/**
 * @brief 云台用，MCU间CAN通讯，CAN回调函数
 * 
 * @param     _hcan 套用
 * @param     RxHead 
 * @param     Data 
 */
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    // 上云台不需要接收自己的信息
    // UP_CLOUD_STATES_CanRx();
    DOWN_CLOUD_STATES_CanRx(RxHead->StdId, Data);
    CHASSIS_STATES_CanRx(RxHead->StdId, Data);
}



/**
 * @brief 设定CAN屏蔽器设定
 * 
 */
void CloudCanFilterConfig()
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
		CanRxCpltCallBack_CloudCommuUpdata(hcan, &bsp_can_Rx, (uint8_t*)CAN_RxData);	//板间CAN通信信息更新
	}
}

