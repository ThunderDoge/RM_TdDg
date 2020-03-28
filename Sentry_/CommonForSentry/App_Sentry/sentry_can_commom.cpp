/**
  * @file  sentry_can_commom.cpp
  * @brief    CAN通讯公共信息
  * @details  
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */    
#include "sentry_can_commom.hpp"

CanCommuRecv_t CanRx,CanTx;	///CAN信息缓存结构体

uint8_t IS_SUPERIOR_VISION_CTRL=0;
uint8_t IS_SUPERIOR_MANUAL_CTRL=0;

///SENTRY_CAN_ID 待匹配列表
uint32_t SENTRY_CAN_ID_List[SENTRY_CAN_ID_CNT] = {
    UP_CLOUD_STATES,                           ///< 上云台状态信息
    DOWN_CLOUD_STATES,                         ///< 下云台状态信息
    CHASSIS_STATES,                            ///< 底盘状态信息
    CHASSIS_PILLAR,                            ///< 底盘撞柱信息（之后会改掉）
    SUPERIOR_UP_RELATIVE_CMD,                  ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_UP_ABSOLUTE_CMD,                  ///< SUPERCMD上云台绝对角度控制
    SUPERIOR_UP_SPEED_CMD,                     ///< 【未启用】SUPERCMD上云台转动速度控制
    SUPERIOR_DOWN_RELATIVE_CMD,                ///< SUPERCMD（上级指令）上云台相对角控制
    SUPERIOR_DOWN_ABSOLUTE_CMD,                ///< SUPERCMD上云台绝对角度控制
    SUPERIOR_DOWN_SPEED_CMD,                   ///< 【未启用】SUPERCMD上云台转动速度控制
    SUPERIOR_CHASSIS_MOVE,                     ///< SUPERCMD底盘速度控制
    SUPERIOR_CHASSIS_SET_LOACTION,             ///< 底盘位置控制
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED, ///< 底盘位置控制加限速
    SUPERIOR_SAFE,                             ///< 底盘安全
    OFFLINE_LIST,                              ///< 离线设备列表
};

// /**
//  * @brief 
//  * 
//  * @param     _hcan CAN句柄
//  * @param     command_id 将要发送的消息的CAN_ID
//  * @param     ptrData 将要发送的消息的字节数组的指针
//  * @param     size 将要发送的消息的字节数
//  * @return HAL_StatusTypeDef 
//  */
// HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData)
// {
//     uint32_t MailBox;
//     CAN_TxHeaderTypeDef bsp_can_Tx;
//     HAL_StatusTypeDef HAL_RESULT;

//     //将传入的数据转换为标准CAN帧数据
//     uint8_t Data[8];
//     if (ptrData != NULL)
//     {
//         memcpy(&Data, ptrData, 8);
//     }
//     else //不用传东西时
//     {
//         memset(&Data, 0, 8);
//     }

//     //设置CAN帧配置
//     bsp_can_Tx.StdId = (uint32_t)command_id;
//     bsp_can_Tx.RTR = CAN_RTR_DATA;
//     bsp_can_Tx.IDE = CAN_ID_STD;
//     bsp_can_Tx.DLC = 8;
//     HAL_RESULT = HAL_CAN_AddTxMessage(_hcan, &bsp_can_Tx, Data, &MailBox);
// #ifndef BSP_CAN_USE_FREERTOS
//     while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//         ; //等待发送完成，如果是使用FreeRTOS则可以不需要这句,因为任务调度本身是需要延时的
// #endif

//     return HAL_RESULT;
// }
/**
 * @brief 
 * 
 * @param     _hcan CAN句柄
 * @param     command_id 将要发送的消息的CAN_ID
 * @param     ptrData 将要发送的消息的字节数组的指针
 * @param     size 将要发送的消息的字节数
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
                                uint8_t *ptrData, size_t size)
{
    uint32_t MailBox;
    CAN_TxHeaderTypeDef bsp_can_Tx;
    HAL_StatusTypeDef HAL_RESULT;

    //将传入的数据转换为标准CAN帧数据
    uint8_t Data[8];
    if (ptrData != NULL)
    {
        memcpy(&Data, ptrData, size);
    }

    //设置CAN帧配置
    bsp_can_Tx.StdId = (uint32_t)command_id;
    bsp_can_Tx.RTR = CAN_RTR_DATA;
    bsp_can_Tx.IDE = CAN_ID_STD;
    bsp_can_Tx.DLC = size;      //设置数据帧消息段长度,即字节数
    HAL_RESULT = HAL_CAN_AddTxMessage(_hcan, &bsp_can_Tx, Data, &MailBox);
#ifndef BSP_CAN_USE_FREERTOS
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
        ; //等待发送完成，如果是使用FreeRTOS则可以不需要这句,因为任务调度本身是需要延时的
#endif
    return HAL_RESULT;
}
/**
 * @brief     CAN发送float型参数
 * 发送两个float
 * @param     _hcan 要发送的CAN的句柄
 * @param     command_id 命令ID，见SENTRY_CAN_ID
 * @param     argu1 第一个float
 * @param     argu2 第二个float
 * @return HAL_StatusTypeDef HAL_OK为正常
 */
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2)
{
    float toSend[2] = {argu1, argu2};
    return SentryCanSend(_hcan, command_id, (uint8_t *)toSend);
}


#ifdef __PROJECT_SENTRY_CLOUD_

#endif // __PROJECT_SENTRY_CLOUD

#ifdef __PROJECT_SENTRY_CHASSIS_
///底盘CAN接收函数
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    CHASSIS_SUPERIOR_ALL_CanRx(RxHead->StdId, Data);
}
#endif // __PROJECT_SENTRY_CHASSIS

/**
 * @brief CAN接收上云台状态。使用时直接在CAN回调函数里面套用
 * 
 * @param     StdId     CAN回调函数的StdId
 * @param     ptrData   CAN回调函数的数据
 */
void UP_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_CLOUD_STATES)
    {
        memcpy(&CanRx.UpCloudPitchYaw, ptrData, 8);
        CanRx.RecvId = UP_CLOUD_STATES;
        CanRx.CanUpdateTime[tUpCloud_Info]=CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_CLOUD_STATES)
        {
            memcpy(&CanRx.DownCloudPitchYaw, ptrData, 8);
            CanRx.RecvId = DOWN_CLOUD_STATES;
            CanRx.CanUpdateTime[tDownCloud_Info]=CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
        }
}
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == CHASSIS_STATES)
	{
        memcpy(&CanRx.Chassis_SpeedLocation, ptrData, 8);	//直接复制两个浮点数进入数组即可
        CanRx.RecvId = CHASSIS_STATES;
        CanRx.CanUpdateTime[tChassis_Info]=CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
	}
}
void CHASSIS_PILLAR_CanRx(uint32_t StdId, uint8_t *ptrData)
{
	if(StdId == CHASSIS_PILLAR)
	{
		CanRx.Pillar_flag = ptrData[0];
        CanRx.CanUpdateTime[tChassis_Info] = CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
	}
}


#ifdef CAN_INTERBOARD

#ifdef __PROJECT_SENTRY_CLOUD_
void UP_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, UP_CLOUD_STATES, CanTx.UpCloudPitchYaw[0],
                  CanTx.UpCloudPitchYaw[1]);
}

void DOWN_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, DOWN_CLOUD_STATES,
                  CanTx.DownCloudPitchYaw[0], CanTx.DownCloudPitchYaw[1]);
}
#endif // __PROJECT_SENTRY_CLOUD_

#ifdef __PROJECT_SENTRY_CHASSIS_
void CHASSIS_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, CHASSIS_STATES,
                  CanTx.Chassis_SpeedLocation[0],
                  CanTx.Chassis_SpeedLocation[1]);
}

void CHASSIS_PILLAR_CanTx()
{
	CanTx.Pillar_flag = ChassisEntity.PillarFlag;
	uint8_t pData[8];
	pData[0] = CanTx.Pillar_flag;
	SentryCanSend(&CAN_INTERBOARD, CHASSIS_PILLAR,pData);
}
#endif // __PROJECT_SENTRY_CHASSIS_

#endif // CAN_INTERBOARD

#ifdef __PROJECT_SENTRY_CLOUD_
/**
 * @brief 云台定时发送的板间CAN通信 - 上云台
 */
void UpCloudCanCommuRoutine(void)
{
// 	if(CurrentMode ==  MODE_VIISON_SHOOTING_TEST)
// 	{
//     CanTx.SuperCon_ChassisMode = VisionRx.chassis_mode;
//     CanTx.SuperCon_ChassisSpeedLocation[0] = VisionRx.Vx;
//     CanTx.SuperCon_ChassisSpeedLocation[1] = VisionRx.Px;
//     CanTx.SuperiorControlFlags = 1;
// 	}
// 	if(GlobalMode == MODE_MANUAL_CHASSIS_MOVE)
//  {
// 		CanTx.SuperCon_ChassisMode = _chassis_speed;
// 		CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_0 * 8000.0f / 660.0f;
// 		CanTx.SuperiorControlFlags = 1;
// 	}
// 	SUPERIOR_CHASSIS_MOVE_CanTx();
//     SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
//     SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx(); //

    CanTx.UpCloudPitchYaw[0] = CloudEntity.RealPitch;   //装载数据
    CanTx.UpCloudPitchYaw[1] = CloudEntity.RealYaw;
    UP_CLOUD_STATES_CanTx();    //发送数据帧
}
/**
 * @brief 云台定时发送的板间CAN通信 - 下云台
 */
void DownCloudCanCommuRoutine(void)
{
    CanTx.UpCloudPitchYaw[0] = CloudEntity.RealPitch;   //装载数据
    CanTx.UpCloudPitchYaw[1] = CloudEntity.RealYaw;
    DOWN_CLOUD_STATES_CanTx();  //发送数据帧
}
#endif  //__PROJECT_SENTRY_CLOUD_
#ifdef __PROJECT_SENTRY_CHASSIS_
/**
 * @brief 底盘定时发送的板间CAN通信
 * 
 */
void ChassisCanCommuRoutine(void)
{
    CanTx.Chassis_SpeedLocation[0] = ChassisEntity.MotorSpeed;
    CanTx.Chassis_SpeedLocation[1] = ChassisEntity.MotorSoftLocation;
    CHASSIS_STATES_CanTx();
	CanTx.Pillar_flag = ChassisEntity.PillarFlag;
	CHASSIS_PILLAR_CanTx();
}
//CAN信息底盘托管控制程序
void ChassisCanRxHandle(void)
{
	ChassisEntity.Mode = (_chassis_mode) CanRx.SuperCon_ChassisMode;
	switch (CanRx.SuperCon_ChassisMode)
	{
	case _chassis_speed:
        ChassisEntity.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location:
        ChassisEntity.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed:
        ChassisEntity.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
		CanRx.Chassis_SpeedLimit);
		break;
	}
}
#endif  //__PROJECT_SENTRY_CHASSIS_

#ifdef CAN_INTERBOARD

void SUPERIOR_UP_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_RELATIVE_CMD,
                  CanTx.SuperCon_PitchYaw[0],
                  CanTx.SuperCon_PitchYaw[1]);
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_ABSOLUTE_CMD,
                  CanTx.SuperCon_PitchYaw[0],
                  CanTx.SuperCon_PitchYaw[1]);
}
void SUPERIOR_DOWN_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_RELATIVE_CMD,
                  CanTx.SuperCon_PitchYaw[0],
                  CanTx.SuperCon_PitchYaw[1]);
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_ABSOLUTE_CMD,
                  CanTx.SuperCon_PitchYaw[0],
                  CanTx.SuperCon_PitchYaw[1]);
}
///底盘运动
void SUPERIOR_CHASSIS_MOVE_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///发送CAN指令控制底盘按位置运动
void SUPERIOR_CHASSIS_SET_LOACTION_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///发送CAN指令，设置底盘控制位置且限速前进
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location_limit_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
///发送CAN安全模式指令
void SUPERIOR_SAFE_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_SAFE, NULL);
}

#endif // CAN_INTERBOARD


#ifdef __PROJECT_SENTRY_CLOUD_

///上云台控制
void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_RELATIVE_CMD)
    {
        memcpy(&CanRx.SuperCon_PitchYaw, ptrData, 8);
        CanRx.SuperCon_CloudMode  = relative_cloud;
        CanRx.CanUpdateTime[tSuperCon_UpCloud] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_ABSOLUTE_CMD)
    {
        memcpy(&CanRx.SuperCon_PitchYaw, ptrData, 8);
        CanRx.SuperCon_CloudMode = absolute_cloud;
        CanRx.CanUpdateTime[tSuperCon_UpCloud] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
//下云台控制
void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_RELATIVE_CMD)
        {memcpy(&CanRx.SuperCon_PitchYaw, ptrData, 8);
        CanRx.CanUpdateTime[tSuperCon_DownCloud] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();}
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_ABSOLUTE_CMD){
        memcpy(&CanRx.SuperCon_PitchYaw, ptrData, 8);
        CanRx.CanUpdateTime[tSuperCon_DownCloud] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
        }

}
#endif // __PROJECT_SENTRY_CLOUD_
#ifdef __PROJECT_SENTRY_CHASSIS_

///底盘运动
void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_MOVE)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_speed;
        CanRx.CanUpdateTime[tSuperCon_Chassis] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
///接收CAN线上指令，设置 模式为CAN控制位置控制，等待执行
void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location;
        CanRx.CanUpdateTime[tSuperCon_Chassis] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
///接收CAN指令，设置底盘控制位置且限速前进
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location_limit_speed;
        CanRx.CanUpdateTime[tSuperCon_Chassis] = CanRx.CanUpdateTime[tSuperiorControl] =CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}

#endif // __PROJECT_SENTRY_CHASSIS_

///接收CAN安全模式指令
void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_SAFE)
    {
        // CanRx.SuperiorControlFlags = 0;
        CanRx.SuperCon_ChassisMode = _chassis_save;
        CanRx.SuperCon_CloudMode = save_cloud;
        CanRx.CanUpdateTime[tSuperiorControl] = CanRx.CanUpdateTime[tCanRecv]=HAL_GetTick();
    }
}
#ifdef __PROJECT_SENTRY_CHASSIS_

///调用全部接收指令的函数
void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId < 0x200)
    {
//        UP_FEED_CanRx(StdId, ptrData);
//        DOWN_FEED_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_MOVE_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(StdId, ptrData);
        SUPERIOR_SAFE_CanRx(StdId, ptrData);
//        CanRx.CanUpdateTime[tCanRecv] = HAL_GetTick(); //Update Timestamp
    }
}
#endif // __PROJECT_SENTRY_CHASSIS_



//供弹。因为结构调整而废弃。

/*
void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_FEED)
        CanRx.feed_flag = *ptrData;
}
void UP_FEED_CanTx()
{
    uint8_t data[8];
    data[0] = CanTx.feed_flag;
    SentryCanSend(&CAN_INTERBOARD, UP_FEED, data);
}
void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_FEED)
        CanRx.feed_flag = *ptrData;
}
void DOWN_FEED_CanTx()
{
    uint8_t data[8];
    data[0] = CanTx.feed_flag;
    SentryCanSend(&CAN_INTERBOARD, DOWN_FEED, data);
}
*/


