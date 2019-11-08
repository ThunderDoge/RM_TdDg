#ifndef	__APP_MOTOR_H_
#define	__APP_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "app_private_math.h"

#define MOTOR6020_POSITION_MIN	0
#define MOTOR6020_POSITION_MAX	8191
#define MOTOR6020_POSITION_PERIOD 8192

#define MOTOR6020_CURRENT_MAX		30000

/*����������ݽṹ��*/
typedef struct 
{
	int16_t RealPosition;	    //!< ��ʵλ��(������)
	int16_t LastPosition;     //!< �ϴ�λ��
	int16_t TargetPosition;	  //!< Ŀ��λ��
	int16_t RealSpeed;		    //!< ʵ���ٶ�(������)
	int16_t TargetSpeed;	    //!< Ŀ���ٶ�
	int16_t RealCurrent;      //!< ��ʵ����(������)
	int16_t TargetCurrent;  	//!< Ŀ�����
	uint8_t RealTemperature;
	uint32_t StartUpdataTime;
}Motor_Data;
typedef struct{
	Motor_Data origin;
	typedef struct 
	{
		Position;
	}soft;
}Motor_Soft;

extern Motor_Data Motor_Yaw_Data, Motor_Pitch_Data;

void app_Motor_Data_Analysis(Motor_Data* motor, uint8_t* Data);
void app_Motor_SoftAnalysis(Motor_Soft* motor,uint8_t* Data);

#endif	//__APP_MOTOR_H_
