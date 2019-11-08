/** 
* @brief    CAN板级支持包
* @details  CAN总线相关设置，数据接收解析函数
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/
#include "bsp_can.h"

//16位大小端转换	用于云台通讯协议解析
#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8)|\
                             (((uint16_t)(A) & 0x00ff) << 8))


Chassis_Control_Data_t Chassis_Control_Data;	//云台主控的控制信息

Motor_Data Motor_Yaw_Data; /*3508电机信息*/
Motor_Data Motor_Pitch_Data;
Motor_Data M3508_Data;
Motor_Data GM6020_Data;

Motor_Data Motor_1_Data;
Motor_Data Motor_2_Data;
Motor_Data Motor_3_Data;
Motor_Data Motor_4_Data;


CAN_TxHeaderTypeDef Bsp_CAN1_Tx;   /*Can总线发送*/
CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can总线接收*/
CAN_TxHeaderTypeDef Bsp_CAN2_Tx;   /*Can2总线发送*/
CAN_RxHeaderTypeDef Bsp_CAN2_Rx;		/*Can2总线接收*/

uint8_t CAN1_RxData[8];       /*CAN1接收缓存数组*/
uint8_t CAN1_TxData[8]={0};   /*CAN1发送数组*/
uint8_t CAN2_RxData[8];       /*CAN2接收缓存数组*/
uint8_t CAN2_TxData[8]={0};   /*CAN2发送数组*/

int16_t CAN1_TxData16[4] = {0};
int16_t CAN2_TxData16[4] = {0};

/**
* @brief  CAN总线配置初始化
* @details  初始化滤波器，数据帧配置
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Init(void)
{
	/*CAN滤波器设置*/
	CAN_FilterTypeDef CAN_FilterConfig;
	
	CAN_FilterConfig.SlaveStartFilterBank=0;
	CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig.FilterIdHigh = 0X0000;
	CAN_FilterConfig.FilterIdLow = 0X0000;
	CAN_FilterConfig.FilterMaskIdHigh = 0X0000;
	CAN_FilterConfig.FilterMaskIdLow = 0X0000;
	CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterConfig.FilterActivation = ENABLE;
	CAN_FilterConfig.FilterBank = 0;
	#ifdef USE_CAN1
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterConfig);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_Start(&hcan1);
	#endif
	#ifdef USE_CAN2
	CAN_FilterConfig.FilterBank = 14;			//位移14
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterConfig);
	HAL_CAN_Start(&hcan2);
	#endif
}

/****
*@func  通用型can发送函数
*@brief 确定id 和发送数据 把数据发送出去 注意此处入口参数是4个int16_t型数据 和8个uint8_t不一样
*@Para
*@Retal
*@data
*******/


HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff)
{
    static CAN_TxHeaderTypeDef	TxHead;//!规定发送的帧格式
		TxHead.RTR=CAN_RTR_DATA;//标准RTR
    TxHead.IDE=CAN_ID_STD;//标准ID不扩展
		TxHead.DLC = 8;
		TxHead.StdId =id;
    static uint8_t Data[8];//中间数组，用于高低位更换
		Data[0] = (uint8_t)((*(s16buff+0)>>8));
		Data[1] = (uint8_t)(*(s16buff+0));
		Data[2] = (uint8_t)((*(s16buff+1)>>8));
		Data[3] = (uint8_t)(*(s16buff+1));
		Data[4] = (uint8_t)((*(s16buff+2)>>8));
		Data[5] = (uint8_t)(*(s16buff+2));
		Data[6] = (uint8_t)((*(s16buff+3)>>8));
		Data[7] = (uint8_t)(*(s16buff+3));
        uint32_t FifoLevel;//当前发送队列的长度
        HAL_StatusTypeDef result=(HAL_CAN_AddTxMessage(_hcan,&TxHead,Data,&FifoLevel));
	return result;
}


/**
* @brief  电机数据加载到CANx_TxData中
* @details  
* @retval  void
*/

void bsp_CAN_TxLoadAllMotorData(void)
{
	
}



/**
* @brief  CAN发送数据
* @details  通过CAN总线发送控制数据
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Sendmessage(void)
{
	uint32_t MailBox;
	/*CANA总线信息发送*/
	//CAN1_TxData[0]=0X0F;
	//CAN1_TxData[1]=0XFF;
	#ifdef USE_CAN1
	HAL_CAN_AddTxMessage(&hcan1, &Bsp_CAN1_Tx, CAN1_TxData, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=3);/*等待发送完成*/
	#endif
	#ifdef USE_CAN2
	HAL_CAN_AddTxMessage(&hcan2, &Bsp_CAN2_Tx, CAN2_TxData, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=3);/*等待发送完成*/
	#endif
	//printf("Mailbox:%d\n\n",MailBox); /*测试发送是否成功*/
}

/** 
  * @brief  检测大小端
  * @retval     1 大端
  * @retval     0 小端
  */
static int Check_Endian()
{
	union{
		uint16_t a;
		uint8_t a_char[2];
	}b;
  b.a = 0x1234;
  return (0x12 == b.a_char[0]);
}


/** 
  * @brief 模拟网络字节序转为本机字节序(短整型)
*/
static uint16_t ntohs(uint16_t a)
{
	return Check_Endian() ? a : BigLittleSwap16(a);
}

/**
* @brief  云台控制信息解析
* @details  
* @param[in]  
* @retval  
*/
void Cloud_Control_calculate(uint8_t* data)
{
//	MastercanTick = xTaskGetTickCount();//获取当前时刻
//	CloudOutofContact = 0;
	memcpy(&Chassis_Control_Data, data, 8);
	
//	if(!SuperCapKeyData.LowPowerFlag)//电容有电
//	{
//		if(Chassis_Control_Data.cmd == ChassisAutoReload)//补弹自动对位
//			InsertCapIndex = 0x07;//自动开300w
//		else if(!(Chassis_Control_Data.RecFlags.SuperCapFlag==0 && (InsertCapIndex&0x0f)==0x01))//若云台命令为不开超级电容而底盘已自动开启则不管云台命令
//			InsertCapIndex = Chassis_Control_Data.RecFlags.SuperCapFlag;
//		InsertCapIndex |= Chassis_Control_Data.RecFlags.SCIAFlag<<4;//电容是否独立接入标志
//	}else InsertCapIndex = 0;//电容没电强制关闭
	
	Chassis_Control_Data.speed_x = ntohs(Chassis_Control_Data.speed_x);
	Chassis_Control_Data.speed_y = ntohs(Chassis_Control_Data.speed_y);
	
//	if(Chassis_Control_Data.cmd != ChassisFollowSecondaryCloud)//底盘跟随二级云台需要特殊处理
		Chassis_Control_Data.speed_z = ntohs(Chassis_Control_Data.speed_z);
//	else{
//		Secondary_Yaw_Position = ntohs(Chassis_Control_Data.speed_z);//此处Secondary_Yaw_Position已是由一级云台板平移过的机械角
//		Secondary_Yaw_Angle = Secondary_Yaw_Position*360/4096;
//	}
//	if(!(chassis_angle_set-Yaw_Gyro_Angle<-30 && Chassis_Control_Data.speed_z>0)
//	&& !(chassis_angle_set-Yaw_Gyro_Angle> 30 && Chassis_Control_Data.speed_z<0))//实际跟随小于30度才继续累积目标角度
//		chassis_angle_set -= (float)Chassis_Control_Data.speed_z/2000;
}

/**
* @brief  电机CAN数据解析到结构体的函数
* @details  
* @param[in]  
* @param[in]  
* @retval  
*/
static void bsp_Motor_CAN_Analysis(Motor_Data* motor, uint8_t* Data)
{
	motor->StartUpdataTime = HAL_GetTick();
	motor->LastPosition = motor->RealPosition;
	motor->RealPosition = Data[0]<<8 | Data[1];
	motor->RealSpeed    = Data[2]<<8 | Data[3];  
	motor->RealCurrent  = Data[4]<<8 | Data[5];
	motor->RealTemperature = Data[6];
}

/**
* @brief  CAN数据解析
* @details  解析CAN接收到的数据包并更新到结构体
* @param  NULL
* @retval  NULL
*/
void bsp_CAN_Analysis(void)
{
	switch(Bsp_CAN1_Rx.StdId)
	{
		case 0x205: bsp_Motor_CAN_Analysis(& Motor_Yaw_Data , CAN1_RxData );	break;
		case 0x206: bsp_Motor_CAN_Analysis(& Motor_Pitch_Data , CAN1_RxData );break;
		case CLOUD_TO_CHASSIS_ID:Cloud_Control_calculate(CAN1_RxData);				break;
	}
	switch(Bsp_CAN2_Rx.StdId )
	{
		case 0x201 : bsp_Motor_CAN_Analysis( & Motor_1_Data , CAN2_RxData );	break;
		case 0x202 : bsp_Motor_CAN_Analysis( & Motor_2_Data , CAN2_RxData  ); break;
		case 0x203 : bsp_Motor_CAN_Analysis( & Motor_3_Data , CAN2_RxData  ); break;
		case 0x204 : bsp_Motor_CAN_Analysis( & Motor_4_Data , CAN2_RxData ); 	break;
	}
}

/**
* @brief  CAN接收中断
* @details  重新定义接收中断，在CAN中断中调用
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static BaseType_t CAN_xHigherPriorityTaskWoken;
	#ifdef USE_CAN1
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)!=0) /*中断处理*/
		{
			HAL_CAN_GetRxMessage(&hcan1, 0, &Bsp_CAN1_Rx, CAN1_RxData);	/*获取CAN报文*/
//			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); /*释放CAN数据更新二值信号量*/
//			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); /*释放CAN数据更新二值信号量*/
 			bsp_CAN_Analysis();

		}
	#endif
	#ifdef USE_CAN2
	if(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0)!=0) /*中断处理*/
		{
			HAL_CAN_GetRxMessage(&hcan2, 0, &Bsp_CAN2_Rx, CAN2_RxData);	/*获取CAN报文*/
//			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); /*释放CAN数据更新二值信号量*/
//			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); /*释放CAN数据更新二值信号量*/
			bsp_CAN_Analysis();
		}
	#endif
}
