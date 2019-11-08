/** 
* @brief    ���˳��õļ���ѧ��������������
* @details  
* @author   ThunderDoge,RM���,2020
* @date      
* @version  
* @par Copyright (c):  ThunderDoge, UESTC
* @par 
*/

#include "app_private_math.h"
/**
* @brief  ��һ�������Ե�ֵ�޶��� [period_low,period_high) ���궨��һ������֮��
* @details  �������޷���
* @param[in]  input	���޶���ֵ
* @param[in]	period_high	��������
* @param[in]	period_low	��������
* @retval  �޶�֮���ֵ�������ֵ���������input
*/

float period_limit(float input, float period_high, float period_low)
{
	if(period_high < period_low)	//����������������Ƿ���ȷ
	{
		float t;
		t = period_high;
		period_high = period_low;
		period_low = t;
	}
	float T = period_high - period_low;			//��input�Ƶ�ָ������֮��
	while(input < period_low)	{input += T;}
	while(input >= period_high) {input -= T;}
	return input;
}
/**
* @brief  һ����ַ�����
* @details  	��������
* @param[in]  
* @retval  
*/

float Limit(float data,float max,float min){
	if(max < min)		//�����޷���
	{
		float tmp = max;
		max = min;
		min = tmp;
	}
	float Temp = data;	//�޷�
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}
