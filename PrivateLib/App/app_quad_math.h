#ifndef __QUAD_MATH_H__
#define __QUAD_MATH_H__

#ifdef  STM32F405xx
#include "stm32f4xx_hal.h"
#define Zero_Sample_Num  2000  //<��������������
#else
#include "stm32f1xx_hal.h"
#define Zero_Sample_Num  800   //<��������������
#endif

#include "app_filter.h"
#include "app_pid.h"


typedef struct 
{
    struct{
      float Gyro[3]; 
      int8_t Data[3][Zero_Sample_Num];   //�����������
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
    }original; //<ԭʼ��
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
    }unitized; //<��λ���� 
    struct{
			float Roll;
			float Pitch;
      float Yaw;
			float StartYaw;
			int16_t YawCircle;
    }soft;	
    float Angle_Rate[3]; 
    float Roll;
    float Pitch;
    float Yaw;
		uint8_t ready;   //�����������һ����������Ե�Ƶˢ��readyΪ0��ʵʱ���9250�Ƿ���
		uint8_t isThisTimeInvalid[3];  // �����ξ�̬���ɼ�ʧ�ܣ�����һ
    		
}MPU_HMC;

extern MPU_HMC imu;
uint8_t IMU_Init(void);
void IMUSO3Thread(void);
#endif
