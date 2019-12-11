#ifndef __app_math_H
#define __app_math_H
#include "stm32f4xx_hal.h"
#include <math.h>
#define PI 		3.14159265358979323846f

#define Threshold_1     8       //��ֵ1����һ�״����˲������仯�Ƕȴ��ڴ�ֵʱ����������
#define Threshold_2     30      //��ֵ2����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����

#define APP_MATH_ABS(x) (x>0?x:-x)

typedef struct
{
	uint8_t last_flag;//�ϴ����ݱ仯����
	uint8_t new_flag;//�������ݱ仯����
	float K_x;//�˲�ϵ��
	uint8_t num_x;
}LPF1;
typedef struct
{
	float out;
}HPF1;
typedef struct 
{
	float _cutoff_freq;
	float _a1;
	float _a2;
	float _b0;
	float _b1;
	float _b2;
	float _delay_element_1;        // buffered sample -1
	float _delay_element_2;        // buffered sample -2	
}LPF2;//�˲����ṹ��

float app_math_Limit(float data,float max,float min);//�޷�����
float app_math_Invsqrt(float number);//��ƽ��������
float app_math_Lpf1apply(LPF1* LPF,float NEW_DATA,float OLD_DATA,float k);
void app_math_Lpf2set(LPF2* LPF,float sample_freq, float cutoff_freq);//�˲�����ֹƵ������
float app_math_Lpf2apply(LPF2* LPF,float sample);//�˲�������
#endif
