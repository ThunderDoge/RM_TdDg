/**
  * @brief    哨兵云台CAN通信
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCanCommu.hpp"
#include <cstring>
//全局CAN接收变量
CanCommuRecv_t CanRecv;
/**
  * @brief  自定义CAN发送函数，由bsp_can
  * @details  
  * @param[in]  _hcan   发送的CAN 的句柄
  * @param[in]  command_id  板间CAN通讯命令ID号
  * @param[in]  ptrData 待发送数据 长度8字节
  * @retval  
  */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, uint8_t *ptrData)
{
    uint32_t MailBox;
    CAN_TxHeaderTypeDef bsp_can_Tx;
    HAL_StatusTypeDef HAL_RESULT;

    //将传入的数据转换为标准CAN帧数据
    uint8_t Data[8];
    // Data[0] = (uint8_t)((*(Can_Send_Data+0)>>8));
    // Data[1] = (uint8_t)(*(Can_Send_Data+0)) & 0XFF;
    // Data[2] = (uint8_t)((*(Can_Send_Data+1)>>8));
    // Data[3] = (uint8_t)(*(Can_Send_Data+1)) & 0XFF;
    // Data[4] = (uint8_t)((*(Can_Send_Data+2)>>8));
    // Data[5] = (uint8_t)(*(Can_Send_Data+2)) & 0XFF;
    // Data[6] = (uint8_t)((*(Can_Send_Data+3)>>8));
    // Data[7] = (uint8_t)(*(Can_Send_Data+3)) & 0XFF;
    //去你妈的大小头，memcpy好
    if (ptrData != NULL)
    {
        memcpy(&Data, ptrData, 8);
    }
    else //不用传东西时
    {
        memset(&Data, 0, 8);
    }

    //设置CAN帧配置
    bsp_can_Tx.StdId = (uint32_t)command_id;
    bsp_can_Tx.RTR = CAN_RTR_DATA;
    bsp_can_Tx.IDE = CAN_ID_STD;
    bsp_can_Tx.DLC = 8;
    HAL_RESULT = HAL_CAN_AddTxMessage(_hcan, &bsp_can_Tx, Data, &MailBox);
#ifndef BSP_CAN_USE_FREERTOS
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
        ; //等待发送完成，如果是使用FreeRTOS则可以不需要这句,因为任务调度本身是需要延时的
#endif

    return HAL_RESULT;
}
/**
  * @brief  发送float型参数的函数包装
  * @details  
  * @param[in]  
  * @retval  
  */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, SENTRY_CAN_ID command_id, float argu1, float argu2)
{
    float toSend[2] = {argu1, argu2};
    return SentryCanSend(_hcan, command_id, (uint8_t *)toSend);
}

void CanRxCpltCallBack_CommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    CanRecv.RecvId = RxHead->StdId;
    switch (RxHead->StdId)
    {
    case SUPERIOR_CHASSIS_MOVE:
        memcpy(&CanRecv.ChassisSpeed, Data, 4);
        memcpy(&CanRecv.ChassisLocation, Data + 4, 4);
        GlobalMode = MODE_VIISON_SHOOTING_TEST;
        CanRecv.SuperiorControlFlags =_SUPERIOR_CHASSIS_SPEED_SET_;
        CanRecv.Ready_Flag =1;
        break;
    case SUPERIOR_CHASSIS_SET_LOACTION:
        memcpy(&CanRecv.ChassisSpeed, Data, 4);
        memcpy(&CanRecv.ChassisLocation, Data + 4, 4);
        GlobalMode = MODE_VIISON_SHOOTING_TEST;
        CanRecv.SuperiorControlFlags =_SUPERIOR_CHASSIS_LOACATION_SET_;
        CanRecv.Ready_Flag =1;
        break;
    case SUPERIOR_SAFE:
        GlobalMode = MODE_SAFE;     //调节到安全模式
        CommandSource = CMDSRC_CAN; //指令源为CAN通信
        CanRecv.SuperiorControlFlags =1;
        CanRecv.Ready_Flag =1;
        break;
    default:
        break;
    }
}
