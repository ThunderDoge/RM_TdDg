#ifndef __APP_IMU_H
#define __APP_IMU_H
#include "stm32f4xx_hal.h"



#define ZERO_SAMPLE_NUM  4000  //零点采样的数据量
typedef struct
{
	struct{
		float Gyro[3]; 
		int16_t Data[3][ZERO_SAMPLE_NUM];   //采样零点数据
		float Sum[3];                    //采样值之和  
		uint16_t Cnt[3];                   //零点计数
		float Mag[3];
	}offset;
	struct{
			int16_t Accel[3]; 
			int16_t Gyro[3];
			int16_t lastGyro[3];
			int16_t Mag[3];        
			int16_t MPU_Temp; 
	}original; //原始的
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
	}unitized; //单位化的 
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
	uint8_t ready;   //建议独立创建一个监控任务，以低频刷新ready为0，实时监测9250是否工作
	uint8_t isThisTimeInvalid[3];  // 若本次静态零点采集失败，则置一
}MPU_DEF;

extern MPU_DEF app_imu_data;

uint8_t app_imu_Init(void);
void app_imu_So3thread(void);
#endif
