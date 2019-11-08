/** 
* @brief    管理文件
* @details  管理数据类型，变量，任务句柄
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/

#ifndef __MY_DEFINE_H_
#define __MY_DEFINE_H_

/*Cube生成的文件*/
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "cmsis_os.h"

/*板级支持包*/
#include "bsp_led.h"
#include "bsp_beep.h"
#include "bsp_can.h"
#include "bsp_dbus.h"
#include "bsp_spi.h"

/*应用层文件*/
#include "Application_pid.h"
#include "Application_oled.h"
#include "Application_math.h"
#include "Application_MPU9250.h"
#include "Application_DT.h"
#include "Application_filter.h"

/*任务文件*/
#include "Logtask.h"
#include "Controltask.h"
#include "Inittask.h"
#include "IMUtask.h"

/*遥控数据结构体*/
typedef struct rc_rec
{
		int16_t CH_0;		/*通道0*/
		int16_t CH_1;		/*通道1*/
		int16_t CH_2;		/*通道2*/
		int16_t CH_3;		/*通道3*/
		uint8_t S1;			/*左开关*/
		uint8_t S2;			/*右开关*/
		int16_t Dial;		/*拨码盘*/
		struct 
		{
			int16_t X;				/*X轴*/
			int16_t Y;				/*Y轴*/
			int16_t Z;				/*Z轴*/
			uint8_t Leftkey;	/*右键*/
			uint8_t Rightkey;	/*右键*/
		}Mouse;					/*鼠标信息*/
		uint16_t Keys;	/*按键信息*/
}RC_Data;

/*电机接收数据结构体*/
typedef struct can_rec
{
		int16_t Loc;		/*电机位置信息*/
		int16_t Rpm;		/*电机转速信息*/
		int16_t Cur;		/*电机电流信息*/
		int8_t Tem;			/*电机温度信息*/
}Motor_Data;

/*电机PID参数结构体*/
typedef struct motor_pid
{
		float Kp;							/*比例项*/
		float Ki;							/*积分项*/
		float Kd;							/*微分项*/
		int16_t I_Limit;			/*积分限制*/
		int16_t Output_Limit;	/*输出限制*/
}Motor_Pid;

/*设备在线检测结构体*/
#define ONLINE 1
#define OFFLINE 0

typedef struct check_online
{
		int Dbus;							/*Dbus离线检测*/
		int CAN;							/*CAN1离线检测*/
		int MPU9250;					/*陀螺仪离线检测*/
}Device_Check;

/*供外部引用的数据和结构体*/
extern Motor_Data M3508_Data; /*3508电机数据接收结构体*/
extern RC_Data Dbus_Data;			/*Dbus遥控信号数据接收结构体*/
extern Motor_Pid M3508_Pid;		/*M3508电机PID控制数据结构体*/

extern uint8_t AK8963_ASA[3];

#define Init_Event	(0X01<<0) //初始化任务事件

/*任务句柄*/
extern TaskHandle_t Log_Task_Handle;	/*系统运行状态任务句柄*/
extern TaskHandle_t Control_Task_Handle;	/*控制任务句柄*/
extern TaskHandle_t Init_Task_Handle;	/*初始化任务句柄*/
extern TaskHandle_t AppTaskCreat_Handle; /*任务创建句柄*/
extern TaskHandle_t IMU_Task_Handle;	/*姿态解算任务句柄*/

extern EventGroupHandle_t Inittask_Event_Handle; /*判断系统初始化是否完成的事件*/
extern SemaphoreHandle_t Dbus_Update_Handle; /*Dbus二值化更新变量*/
extern SemaphoreHandle_t CAN_Update_Handle; /*CAN二值化更新变量*/
extern SemaphoreHandle_t Dbus_Check_Handle; /*Dbus离线检测二值信号量*/
extern SemaphoreHandle_t CAN_Check_Handle;	/*CAN离线检测二值信号量*/

#endif
