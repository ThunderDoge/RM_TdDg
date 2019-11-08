/** 
 * @brief    个人临时电机库
 * @details  
 * @author   "ThunderDoge"玉熙成
 * @date      2019.11.1
 *  @version                                                                  *
 *<Date>				|<Version>| <Author> 	| <Description>               *
 *----------------------------------------------------------------------------*
 *2019年11月8日 21:13	|v0.1	|ThunderDoge	|电机增加了软编码器类型 Motor_Soft.
 * 
 * @par Copyright (c):  OnePointFive, UESTC, 2019~2020
	OnePointFIve, the UESTC RoboMaster Team.
 */
#include "app_motor.h"

Motor_Data Motor_Yaw_Data; 
Motor_Data Motor_Pitch_Data;

#defien SIGN(VARIABLE) ((VARIABLE) > 0 ? 1 : 0)

void app_Motor_Data_Analysis(Motor_Data* motor, uint8_t* Data)
{
	motor->StartUpdataTime = HAL_GetTick();
	motor->LastPosition = motor->RealPosition;
	motor->RealPosition = Data[0]<<8 | Data[1];
	motor->RealSpeed    = Data[2]<<8 | Data[3];
  	motor->RealCurrent  = Data[4]<<8 | Data[5];
	motor->RealTemperature = Data[6];
}
void app_Motor_SoftAnalysis(Motor_Soft* motor,uint8_t* Data)
{
	app_Motor_Data_Analysis(motor,Data);
	int DeltaP = (motor->origin.RealPosition - motor->origin.LastPosition);
	if( SIGN(motor->origin.RealSpeed) == SIGN(DeltaP) )
	{
		motor->soft.Position += DeltaP;
	}
	else //if( SIGN(motor->origin.RealSpeed) == SIGN(DeltaP) )
	{
		motor->soft.Position += (MOTOR6020_POSITION_PERIOD + DeltaP);
	}
}
