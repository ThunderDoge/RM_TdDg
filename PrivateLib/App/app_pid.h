#ifndef __MY_PID_H
#define __MY_PID_H

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "can.h"

#define BLOCK_INT_LB	-2000	//Blocking Integral Left Bound
#define	BLOCK_INT_RB	2000	//Blocking Integral Right Bound

#define MOTOR_POSITION_MIN	0
#define MOTOR_POSITION_MAX	8191

#define	MOTOR_CURRENT_MIN		-30000
#define MOTOR_CURRENT_MAX		30000

//#define USE_SLIDING_FILTER
#define SLIDING_BUFFER_LENGTH	50

typedef struct{
	uint16_t window_lenght;
	uint16_t buffer_loaded_num;
	int16_t slide_buffer[SLIDING_BUFFER_LENGTH];
	uint16_t buffer_iter_start, buffer_iter_end;
	int32_t	sum_of_buffer;
	float output;
}SlidingFilter;


typedef struct
{
	float u_t;
	float K_p;
	float K_i;
	float K_d;
	float e_t;
	float prev_e_t;
	float integ_of_et;
	float diff_of_et;
	
	
	float eIntegralAbsMax;			//�������Ʒ���
	float OutputAbsMax;					//������Ʒ���
	
	float IntegralNeighbourAbs;	//���ַ�Χ����ֵ

	int16_t* pMtLoad;	//����ֵ����ָ��
	int16_t* pRtLoad;	//Ŀ��ֵ����ָ��
	int16_t* pUtSave;	//���ֵ����ָ�룬���������û�����
	
	#ifdef USE_SLIDING_FILTER
	SlidingFilter sf;	//�����˲�
	#endif
	
}PID_t;

//typedef struct 
//{
//	PID_t* pPID;			//ָ��PID��ָ��
//	
//	int16_t* pRt;			//����ֵ����ָ��
//	int16_t* pMt;			//����ֵ����ָ��
//	int16_t* pUt;			//���ֵ����ָ��
//	
//	float Kp_t;				
//	float Ki_t;
//	float Kd_t;
//	
//	int16_t eIntegralMax;
//	int16_t	eIntegralMin;
//	
//	int16_t OutputMax;
//	int16_t	OutputMin;
//	
//}PID_InitTypeDef;		//PID ��ʼ���ṹ��

//PIDȫ�ֱ���
//extern PID_t PID_0;
//extern PID_t PID_Speed,PID_Angle;
extern PID_t PID_Cloud_Yaw, PID_Cloud_Yaw_Speed, PID_Cloud_Pitch, PID_Cloud_Pitch_Speed ;

//void PID_Init(PID_InitTypeDef* PID_InitStructure);
void PID_Init( PID_t* pPID , int16_t* pRt , int16_t* pMt , int16_t* pUt ,
								float Kp_t , float Ki_t , float Kd_t, int16_t eIntegralMax , int16_t OutputMax);
void PID_Init_rt(PID_t* pPID , 
								float Kp_t , float Ki_t , float Kd_t, int16_t eIntegralMax , int16_t OutputMax);
void PID_SetWindowLength(PID_t* pPID , uint16_t window_length);
void Motor_PID_Loop_Section(PID_t* pPID); 
void Motor_PID_Loop_Section_rt( PID_t* pPID, float Target, float Measure, float* Output);
void PID_MixedFeedback_rt( PID_t* pPID, float Target, float Measure, float RateOfErr, float* Output );


#endif
