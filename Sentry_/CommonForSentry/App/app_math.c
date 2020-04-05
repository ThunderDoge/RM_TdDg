/** 
 * @file app_math.c
* @brief        ��ѧ��� 
* @details  
* @author    Nankel  Li, Asn, Thunderdoge
* @date     2019.10.16 
* @version  0.2
* @par Copyright (c):  RM2020���
*  
* @par ��־
    2020-4-1    Thunderdoge ������������޷�����
*/  
#include "app_math.h"
#include <math.h>

///** 
//* @brief   �޷����� 
//* @remarks min<data<max
//*/
float app_math_Limit(float data,float max,float min)
{
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}

/** 
* @brief   ��ƽ�����ĵ���
* @remarks ʹ�þ����Carmack�㷨��Ч�ʸߣ���������µ�����
*/
float app_math_Invsqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5f;

    x = number * 0.5f;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/*****��ϵ���޸ĵ�һ���˲�����

��ڣ�LPF         �˲����ṹ��
			NEW_DATA    �²�����ֵ
      OLD_DATA    �ϴ��˲���õĽ��
      k           �˲�ϵ��(�������˲�����е�Ȩ��)
���ڣ� result      �����˲����
 */
float app_math_Lpf1apply(LPF1* LPF,float NEW_DATA,float OLD_DATA,float k)
{
    //�Ƕȱ仯����new_flag=1��ʾ�Ƕ����ӣ�=0��ʾ�Ƕ����ڼ�С
    if((NEW_DATA-OLD_DATA)>0)
        LPF->new_flag=1;
    else if((NEW_DATA-OLD_DATA)<0)
        LPF->new_flag=0;


    if(LPF->new_flag==LPF->last_flag)  //�˴α仯��ǰһ�α仯�����Ƿ�һ�£���ȱ�ʾ�Ƕȱ仯����һ��
        {
            LPF->num_x++;
            if(fabs((NEW_DATA-OLD_DATA))>Threshold_1)
        //���仯����Threshold_1��ʱ�򣬽��м�����num�������ӣ��Դﵽ��������Kֵ����߸�����
                LPF->num_x+=5;                           

            if(LPF->num_x>Threshold_2)   //������ֵ���ã������ݵ�����ݼ��ٶȴﵽһ������ʱ������Kֵ
            {
                LPF->K_x=k+0.2f;          //0.2ΪK_x������ֵ����ʵ����Ҫ�޸�
                LPF->num_x=0;
            }
        }
    else 
        {
            LPF->num_x=0;
            LPF->K_x=0.01f;     //���ݱ仯�ȶ�ʱK_xֵ����ʵ���޸�
        }
		LPF->last_flag = LPF->new_flag;
    OLD_DATA=(1-LPF->K_x)*OLD_DATA+LPF->K_x*NEW_DATA;
    return OLD_DATA;
}
/** 
* @brief  һ�׸�ͨ�˲���
* @param[in]   inArgName input argument description. 
* @retval  ��
* @par ��־ 
*
*/
void app_math_hpf1apply(HPF1* HPF,float sample_freq, float cutoff_freq)
{
}

/** 
* @brief    ���׵�ͨ�˲���
* @remarks 
* @par ��־
*/
void app_math_Lpf2set(LPF2* LPF,float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(PI/fr);
		c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
	
    LPF->_cutoff_freq = cutoff_freq;
    if (LPF->_cutoff_freq > 0.0f) 
		{
				LPF->_b0 = ohm*ohm/c;
				LPF->_b1 = 2.0f*LPF->_b0;
				LPF->_b2 = LPF->_b0;
				LPF->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
				LPF->_a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
		}
}

float app_math_Lpf2apply(LPF2* LPF,float sample)
{
		float delay_element_0 = 0, output=0;	
    if (LPF->_cutoff_freq <= 0.0f) {
        // no mathing
        return sample;
    }
		else
		{
				delay_element_0 = sample - LPF->_delay_element_1 * LPF->_a1 - LPF->_delay_element_2 * LPF->_a2;
				// do the mathing
				if (isnan(delay_element_0) || isinf(delay_element_0)) 
						// don't allow bad values to propogate via the math
						delay_element_0 = sample;

				output = delay_element_0 * LPF->_b0 + LPF->_delay_element_1 * LPF->_b1 + LPF->_delay_element_2 * LPF->_b2;
				
				LPF->_delay_element_2 = LPF->_delay_element_1;
				LPF->_delay_element_1 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}


float app_math_fLimitPeriod(float data,float max,float min)
{
    float period = max - min;

    while(data > max)
    {
        data -= period;
    }

    while (data < min)
    {
        data += period;
    }

    return data;
}









