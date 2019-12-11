#ifndef __app_math_H
#define __app_math_H
#include "stm32f4xx_hal.h"
#include <math.h>
#define PI 		3.14159265358979323846f

#define Threshold_1     8       //阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
#define Threshold_2     30      //阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随

#define APP_MATH_ABS(x) (x>0?x:-x)

typedef struct
{
	uint8_t last_flag;//上次数据变化方向
	uint8_t new_flag;//本次数据变化方向
	float K_x;//滤波系数
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
}LPF2;//滤波器结构体

float app_math_Limit(float data,float max,float min);//限幅函数
float app_math_Invsqrt(float number);//开平方根函数
float app_math_Lpf1apply(LPF1* LPF,float NEW_DATA,float OLD_DATA,float k);
void app_math_Lpf2set(LPF2* LPF,float sample_freq, float cutoff_freq);//滤波器截止频率设置
float app_math_Lpf2apply(LPF2* LPF,float sample);//滤波器设置
#endif
