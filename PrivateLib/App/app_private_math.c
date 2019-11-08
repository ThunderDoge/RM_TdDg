/** 
* @brief    个人常用的简单数学处理，集中在这里
* @details  
* @author   ThunderDoge,RM电控,2020
* @date      
* @version  
* @par Copyright (c):  ThunderDoge, UESTC
* @par 
*/

#include "app_private_math.h"
/**
* @brief  将一个周期性的值限定在 [period_low,period_high) 所标定的一个周期之内
* @details  有上下限防呆
* @param[in]  input	被限定的值
* @param[in]	period_high	周期上限
* @param[in]	period_low	周期下限
* @retval  限定之后的值，将这个值赋给外面的input
*/

float period_limit(float input, float period_high, float period_low)
{
	if(period_high < period_low)	//防呆，检查上下限是否正确
	{
		float t;
		t = period_high;
		period_high = period_low;
		period_low = t;
	}
	float T = period_high - period_low;			//将input移到指定周期之内
	while(input < period_low)	{input += T;}
	while(input >= period_high) {input -= T;}
	return input;
}
/**
* @brief  一般的现辐函数
* @details  	附带防呆
* @param[in]  
* @retval  
*/

float Limit(float data,float max,float min){
	if(max < min)		//上下限防呆
	{
		float tmp = max;
		max = min;
		min = tmp;
	}
	float Temp = data;	//限幅
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}
