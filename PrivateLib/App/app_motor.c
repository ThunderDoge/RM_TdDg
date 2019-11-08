/** 
* @brief    个人临时电机库
* @details  
* @author   "ThunderDoge"玉熙成
* @date      2019.11.1
* @version  v0.1
* @par Copyright (c):  OnePointFive, UESTC, 2019~2020
	OnePointFIve, the UESTC RoboMaster Team.
*/
#include "app_motor.h"

Motor_Data Motor_Yaw_Data; 
Motor_Data Motor_Pitch_Data;



void app_Motor_Data_Analysis(Motor_Data* motor, uint8_t* Data)
{
	motor->StartUpdataTime = HAL_GetTick();
	motor->LastPosition = motor->RealPosition;
	motor->RealPosition = Data[0]<<8 | Data[1];
	motor->RealSpeed    = Data[2]<<8 | Data[3];  
  motor->RealCurrent  = Data[4]<<8 | Data[5];
	motor->RealTemperature = Data[6];
}
