/** 
* @file         app_imu.c
* @brief        ʹ��icm20602��Ϊapp_imu_data
* @details  
* @author      Asn
* @date     2019��11��17��
* @version  
* @par Copyright (c):  RM2020���
*  
* @par ��־
*	2019.11.17 Asn V1.0 ��ֲmpu9250��

*						 Asn V1.1 ����mahony�㷨��PI���Ʋ���

*						 Asn V1.2 ����֮ǰ������Ӳ���˲������д���������

*	2019.11.28 Asn V2.0 ���Ѽ�������ƣ����Ż����㷨���ӿ��˳�ʼ�������ٶ�

*						 Asn V2.1 ��ʽ����app_filterΪapp_math������limit��invsqrt�ƽ�ȥ

*	2019.12.6  Asn V2.2 �Ż�PI���������������������Ԥ���������ٶ�ģ��������С���������

*	2019.12.11 Asn V2.3 ������soft�ǶȽ�������������һ��ʼ�����������ۻ�Ȧ�����������������룬������ô����Ƶľ���
* 2019.12.24 Asn V2.4 �����˼��ٶȼ��˲�����
*   2020-3-18   ThunderDoge �ڱ������������������߼�⺯��
*/  
#ifndef __APP_IMU_H
#define __APP_IMU_H
#include "stm32f4xx_hal.h"
#include "app_sentry_check_device.hpp"




#define ZERO_SAMPLE_NUM  4000  //��������������
#define APP_MATH_ABS(x)   ((x)>0?(x):-(x))
typedef struct
{
	struct{
		float Gyro[3]; 
		int16_t Data[3][ZERO_SAMPLE_NUM];   //�����������
		float Sum[3];                    //����ֵ֮��  
		uint16_t Cnt[3];                   //������
		float Mag[3];
	}offset;
	struct{
			int16_t Accel[3]; 
			int16_t Gyro[3];
			int16_t lastGyro[3];
			int16_t Mag[3];        
			int16_t MPU_Temp; 
	}original; //ԭʼ��
	struct{
			float Accel[3]; 
			float Gyro[3];
			float Mag[3];        
	}kalman; 
	struct{
			float Accel[3]; 
			float Gyro[3];
			float Mag[3];  
	}LPF;			
	struct{
			float Accel[3]; 
			float Gyro[3];
			float Mag[3];        
			float MPU_Temp; 
	}unitized; //��λ���� 
	struct{
		float Roll;
		float Pitch;
		float Yaw;
	}soft;
	struct{
		float Roll;
		float Pitch;
		float Yaw;
	}integral;
	float Angle_Rate[3]; 
	float Roll;
	float Pitch;
	float Yaw;
	uint8_t reset;
	uint8_t ready;   //�����������һ����������Ե�Ƶˢ��readyΪ0��ʵʱ���9250�Ƿ���
	uint8_t isThisTimeInvalid[3];  // �����ξ�̬���ɼ�ʧ�ܣ�����һ
}MPU_DEF;

// ���߼�� �ṹ��
extern struct CheckDevice_Type IMU_CheckDevice;


extern MPU_DEF app_imu_data;

extern uint32_t tNow;   //app_imu_So3thread����ʱ��΢��(us)��������Hal_GetTick()*1000

uint8_t app_imu_Init(void);
void app_imu_So3thread(void);
#endif
