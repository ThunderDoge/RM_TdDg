#ifndef	__APP_MOTOR_H_
#define	__APP_MOTOR_H_

#include "stm32f4xx_hal.h"

#define MOTOR6020_POSITION_MIN	0
#define MOTOR6020_POSITION_MAX	8191

#define MOTOR6020_CURRENT_MAX		30000

/*电机接收数据结构体*/
typedef struct 
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

extern Motor_Data Motor_Yaw_Data, Motor_Pitch_Data;

#endif	//__APP_MOTOR_H_
