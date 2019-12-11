#ifndef __APP_IMU_H
#define __APP_IMU_H
#include "stm32f4xx_hal.h"



#define ZERO_SAMPLE_NUM  4000  //��������������
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

extern MPU_DEF app_imu_data;

uint8_t app_imu_Init(void);
void app_imu_So3thread(void);
#endif
