#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"

//宏定义用到的CAN
#define USE_CAN1
#define USE_CAN2

#define CLOUD_TO_CHASSIS_ID  0x101			//底盘CAN通信ID

extern CAN_TxHeaderTypeDef Bsp_CAN1_Tx;   /*Can总线发送*/
extern CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can总线接收*/
extern CAN_TxHeaderTypeDef Bsp_CAN2_Tx;   /*Can2总线发送*/
extern CAN_RxHeaderTypeDef Bsp_CAN2_Rx;		/*Can2总线接收*/


extern uint8_t CAN1_RxData[8];       /*CAN1接收缓存数组*/
extern uint8_t CAN1_TxData[8];   /*CAN1发送数组*/
extern uint8_t CAN2_RxData[8];       /*CAN2接收缓存数组*/
extern uint8_t CAN2_TxData[8];   /*CAN2发送数组*/
extern int16_t CAN1_TxData16[4];
extern int16_t CAN2_TxData16[4];


extern int16_t CAN1_ReLoc; /*CAN总线接收的位置数据*/
extern int16_t CAN1_ReRpm; /*CAN总线接收的转速数据*/

typedef struct{
		uint8_t SuperCapFlag :4;//超级电容的使用开关 1为使用
		uint8_t ReverseFlag :1;//底盘反向标志 1为反向
		uint8_t SCIAFlag :1;//电容独立接入底盘标志 1为独立接入
		uint8_t AutoReloadCalibrationFlag : 1;//自动对位校准标志
		uint8_t CloudJudgementOutOfContact : 1;//云台裁判系统失联
//		uint8_t LaserRadarMode_ON : 1;//启用激光雷达模式标志
}RecFlags_t;//各项标志


typedef struct//收(CLOUD_TO_CHASSIS_ID)
{
	uint8_t cmd;//底盘运动模式
	RecFlags_t RecFlags;//各种标志，1有效，[0-3]超级电容；[4]底盘反向
	int16_t speed_x;
	int16_t speed_y;
	int16_t speed_z;
}Chassis_Control_Data_t;
extern Chassis_Control_Data_t Chassis_Control_Data;//云台主控的控制信息


/*电机接收数据结构体*/
typedef struct can_rec
{
	int16_t RealPosition;	    //!< 真实位置(编码器)
	int16_t LastPosition;     //!< 上次位置
	int16_t TargetPosition;	  //!< 目标位置
	int16_t RealSpeed;		    //!< 实际速度(编码器)
	int16_t TargetSpeed;	    //!< 目标速度
	int16_t RealCurrent;      //!< 真实电流(编码器)
	int16_t TargetCurrent;  	//!< 目标电流
	uint8_t RealTemperature;
	uint32_t StartUpdataTime;
}Motor_Data;


extern Motor_Data Motor_Yaw_Data; 	//全局电机变量
extern Motor_Data Motor_Pitch_Data;
extern Motor_Data Motor_1_Data;
extern Motor_Data Motor_2_Data;
extern Motor_Data Motor_3_Data;
extern Motor_Data Motor_4_Data;


void bsp_CAN_Init(void);
void bsp_CAN_TxLoadAllMotorData(void);
void bsp_CAN_Sendmessage(void);
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff);
void bsp_CAN_Analysis(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
