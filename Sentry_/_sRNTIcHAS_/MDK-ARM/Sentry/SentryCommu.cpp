#include "SentryCommu.hpp"

// Infomation Storage
CanCommuRecv_t CanInfo, CanRx, CanTx;
#ifdef USE_VISION
Sentry_vision_data VisionRx, VisionTx;
#endif

// CAN TxRx Functions
//状态广播
void UP_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_CLOUD_STATES)
        memcpy(&CanRx.UpCloudPitchYaw, ptrData, 8);
}
void UP_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, UP_CLOUD_STATES, CanTx.UpCloudPitchYaw[0],
                  CanTx.UpCloudPitchYaw[1]);
}

void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_CLOUD_STATES)
        memcpy(&CanRx.DownCloudPitchYaw, ptrData, 8);
}
void DOWN_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, DOWN_CLOUD_STATES,
                  CanTx.DownCloudPitchYaw[0], CanTx.DownCloudPitchYaw[1]);
}
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == CHASSIS_STATES)
	{
        memcpy(&CanRx.Chassis_SpeedLocation, ptrData, 8);	//直接复制两个浮点数进入数组即可
	}
}
void CHASSIS_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, CHASSIS_STATES,
                  CanTx.Chassis_SpeedLocation[0],
                  CanTx.Chassis_SpeedLocation[1]);
//	SentryCanSend(&CAN_INTERBOARD, CHASSIS_STATES,
//                  CanTx.Chassis_SpeedLocation[0],
//                  CanTx.Chassis_SpeedLocation[1]);
}
void CHASSIS_PILLAR_CanRx(uint32_t StdId, uint8_t *ptrData)
{
	if(StdId == CHASSIS_PILLAR)
	{
		CanRx.Pillar_flag = ptrData[0];
	}
}
void CHASSIS_PILLAR_CanTx()
{
	CanTx.Pillar_flag = Self.PillarFlag;
	uint8_t pData[8];
	pData[0] = CanTx.Pillar_flag;
	SentryCanSend(&CAN_INTERBOARD, CHASSIS_PILLAR,pData);
}

//上级命令广播
//上云台控制
void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_RELATIVE_CMD)
        memcpy(&CanRx.SuperCon_Relative_PitchYaw, ptrData, 8);
}
void SUPERIOR_UP_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_RELATIVE_CMD,
                  CanTx.SuperCon_Relative_PitchYaw[0],
                  CanTx.SuperCon_Relative_PitchYaw[1]);
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_ABSOLUTE_CMD)
        memcpy(&CanRx.SuperCon_Absolute_PitchYaw, ptrData, 8);
}
void SUPERIOR_UP_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_ABSOLUTE_CMD,
                  CanTx.SuperCon_Absolute_PitchYaw[0],
                  CanTx.SuperCon_Absolute_PitchYaw[1]);
}
//下云台控制
void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_RELATIVE_CMD)
        memcpy(&CanRx.SuperCon_Relative_PitchYaw, ptrData, 8);
}
void SUPERIOR_DOWN_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_RELATIVE_CMD,
                  CanTx.SuperCon_Relative_PitchYaw[0],
                  CanTx.SuperCon_Relative_PitchYaw[1]);
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_DOWN_ABSOLUTE_CMD)
        memcpy(&CanRx.SuperCon_Absolute_PitchYaw, ptrData, 8);
}
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_DOWN_ABSOLUTE_CMD,
                  CanTx.SuperCon_Absolute_PitchYaw[0],
                  CanTx.SuperCon_Absolute_PitchYaw[1]);
}

//供弹
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
//底盘运动
void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_MOVE)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_speed;
    }
}
void SUPERIOR_CHASSIS_MOVE_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_MOVE,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}

void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location;
    }
}
void SUPERIOR_CHASSIS_SET_LOACTION_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}

void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED)
    {
        memcpy(&CanRx.SuperCon_ChassisSpeedLocation, ptrData, 8);
        CanRx.SuperCon_ChassisMode = _chassis_location_limit_speed;
    }
}
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx()
{
    if (CanTx.SuperCon_ChassisMode == _chassis_location_limit_speed)
        SentryCanSend(&CAN_INTERBOARD, SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED,
                      CanTx.SuperCon_ChassisSpeedLocation[0],
                      CanTx.SuperCon_ChassisSpeedLocation[1]);
}
//安全模式
void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_SAFE)
    {
        CanRx.SuperiorControlFlags = 0;
        CanRx.SuperCon_ChassisMode = _chassis_save;
    }
}
void SUPERIOR_SAFE_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_SAFE, NULL);
}
void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId < 0x200)
    {
        UP_FEED_CanRx(StdId, ptrData);
        DOWN_FEED_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_MOVE_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_CanRx(StdId, ptrData);
        SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(StdId, ptrData);
        SUPERIOR_SAFE_CanRx(StdId, ptrData);
        CanRx.RecvUpdateTime = HAL_GetTick(); //Update Timestamp
    }
}
#ifdef USE_VISION
//VisionUart TxRx Functions 视觉串口函数
//VisionUart Recv 视觉串口接收函数
/**
  * @brief  云台相对角度控制
  * @details  
  */
void CMD_GIMBAL_RELATIVE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_RELATIVE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 2, 4); //云台角度解析
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1); //云台模式解析
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1); //射击模式
        VisionRx.cloud_ctrl_mode = relative_cloud;                  //数据就绪
    }
}
/**
  * @brief  云台绝对角度控制
  */
void CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_ABSOLUTE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1);
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1);
        VisionRx.cloud_ctrl_mode = absolute_cloud; //数据就绪
    }
}
/**
  * @brief  射击控制
  */
void CMD_SHOOT_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_SHOOT)
    {
        VisionRx.Function_word = CMD_SHOOT;
        VisionRx.Shoot_mode = 1;                               //射击指令就绪
        memcpy(&VisionRx.Shoot_speed, Vision_Rxbuffer + 2, 4); //射击速度
        memcpy(&VisionRx.Shoot_freq, Vision_Rxbuffer + 6, 1);  //射击频率
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 7, 1);  //射击模式
    }
}
/**
  * @brief  底盘运动控制
  */
void CMD_CHASSIS_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_CONTROL)
    {
        bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
        memcpy(&bsp_vision_Rec_Data.Vx, Vision_Rxbuffer + 2, 4); //底盘速度解析
        memcpy(&bsp_vision_Rec_Data.Vy, Vision_Rxbuffer + 6, 4);
    }
}
/**
  * @brief  底盘路程控制
  */
void CMD_CHASSIS_LOACTION_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_LOACTION_CONTROL)
    {
        VisionRx.Function_word = CMD_CHASSIS_LOACTION_CONTROL;
        memcpy(&VisionRx.Px, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Py, Vision_Rxbuffer + 6, 4);
        VisionRx.chassis_mode = _chassis_location;
    }
}
void CMD_CHASSIS_LOCATION_LIMIT_SPEED_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_LOCATION_LIMIT_SPEED)
    {
        VisionRx.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
        memcpy(&VisionRx.Px, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.SpeedLimit, Vision_Rxbuffer + 6, 4);
        VisionRx.chassis_mode = _chassis_location_limit_speed;
    }
}
/**
  * @brief  全命令接收
  */
//视觉串口中断接收函数
void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer)
{
    CMD_GIMBAL_RELATIVE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_SHOOT_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOACTION_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOCATION_LIMIT_SPEED_Rx(Vision_Rxbuffer);
}
//视觉串口发送函数
void CMD_GET_MCU_STATE_Tx()
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer((uint8_t)VisionTx.Cloud_mode, 0U);
    bsp_vision_load_to_txbuffer(VisionTx.Pitch, 1);
    bsp_vision_load_to_txbuffer(VisionTx.Yaw, 5);
    bsp_vision_load_to_txbuffer(VisionTx.Shoot_speed, 9U);
    bsp_vision_load_to_txbuffer(VisionTx.Shoot_mode, 13U);
    bsp_vision_SendTxbuffer(CMD_GET_MCU_STATE);
}
void ROBOT_ERR_Tx()
{
    	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer(VisionTx.Error_code, 0U);
    bsp_vision_SendTxbuffer(ROBOT_ERR);
}
void STA_CHASSIS_Tx()
{
    		while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer(VisionTx.chassis_mode, 0U);
    bsp_vision_load_to_txbuffer(VisionTx.pillar_flag, 1U);
    bsp_vision_load_to_txbuffer(VisionTx.Vx, 2);
	bsp_vision_load_to_txbuffer(VisionTx.Px, 6);
    bsp_vision_SendTxbuffer(STA_CHASSIS);
}

void VisionRxHandle(void)
{
    //云台指令处理
    switch (VisionRx.cloud_ctrl_mode)
    {
    case relative_cloud:
        Self.SetAngleTo(Self.RealPitch + VisionRx.Pitch,
                        Self.RealYaw + VisionRx.Yaw);
        break;
    case absolute_cloud:
        Self.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
    default:
        break;
    }
    VisionRx.cloud_ctrl_mode = 0;    //处理完成标志。因为一个命令只会处理一次，处理后置0
}
void CloudVisonTxRoutine(void)
{
    VisionTx.Cloud_mode = 0;
    VisionTx.Shoot_mode = 0;
    VisionTx.Pitch = Self.RealPitch;
    VisionTx.Yaw = Self.MechanicYaw;
    VisionTx.Shoot_speed = 0;

    VisionTx.Error_code = 0;

    VisionTx.chassis_mode = VisionRx.chassis_mode;
    VisionTx.Vx = CanRx.Chassis_SpeedLocation[0];
    VisionTx.pillar_flag = CanRx.Pillar_flag;
    VisionTx.Px = CanRx.Chassis_SpeedLocation[1];

    CMD_GET_MCU_STATE_Tx();
    ROBOT_ERR_Tx();
    STA_CHASSIS_Tx();
}
#endif
void CanRxCpltCallBack_CloudCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    // 上云台不需要接收自己的信息
    // UP_CLOUD_STATES_CanRx();
    DOWN_CLOUD_STATES_CanRx(RxHead->StdId, Data);
    CHASSIS_STATES_CanRx(RxHead->StdId, Data);
}
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data)
{
    CHASSIS_SUPERIOR_ALL_CanRx(RxHead->StdId, Data);
}
#ifdef CLOUD_COMMMU
void CloudCanCommuRoutine(void)
{
    	if(GlobalMode ==  MODE_VIISON_SHOOTING_TEST)
	{
    CanTx.SuperCon_ChassisMode = VisionRx.chassis_mode;
    CanTx.SuperCon_ChassisSpeedLocation[0] = VisionRx.Vx;
    CanTx.SuperCon_ChassisSpeedLocation[1] = VisionRx.Px;
    CanTx.SuperiorControlFlags = 1;
	}
	if(GlobalMode == MODE_MANUAL_CHASSIS_MOVE)
	{
		CanTx.SuperCon_ChassisMode = _chassis_speed;
		CanTx.SuperCon_ChassisSpeedLocation[0] = bsp_dbus_Data.CH_0 * 5000.0f / 660.0f;
    CanTx.SuperiorControlFlags = 1;
    }
    SUPERIOR_CHASSIS_MOVE_CanTx();
    SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx(); //
}
#endif
#ifdef CHASSIS_COMMU
void ChassisCanCommuRoutine(void)
{
    CanTx.Chassis_SpeedLocation[0] = Self.RealSpeed;
    CanTx.Chassis_SpeedLocation[1] = Self.RealPosition;
    CHASSIS_STATES_CanTx();
}
//CAN信息底盘托管控制程序
void ChassisCanRxHandle(void)
{
	Self.Mode = (_chassis_mode) CanRx.SuperCon_ChassisMode;
	switch (CanRx.SuperCon_ChassisMode)
	{
	case _chassis_speed:
        Self.MotorSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[0]);
        break;
    case _chassis_location:
        Self.MotorSoftLocation_Set(CanRx.SuperCon_ChassisSpeedLocation[1]);
        break;
    case _chassis_location_limit_speed:
        Self.MotorSoftLocation_LimitSpeed_Set(CanRx.SuperCon_ChassisSpeedLocation[1],
		CanRx.Chassis_SpeedLimit);
		break;
	}
}
#endif
