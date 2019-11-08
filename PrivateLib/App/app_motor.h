#ifndef	__APP_MOTOR_H_
#define	__APP_MOTOR_H_

#include "stm32f4xx_hal.h"

#define MOTOR6020_POSITION_MIN	0
#define MOTOR6020_POSITION_MAX	8191

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

extern Motor_Data Motor_Yaw_Data, Motor_Pitch_Data;

#endif	//__APP_MOTOR_H_
