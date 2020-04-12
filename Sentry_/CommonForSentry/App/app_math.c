/** 
 * @file app_math.c
* @brief        数学相关 
* @details  
* @author    Nankel  Li, Asn, Thunderdoge
* @date     2019.10.16 
* @version  0.2
* @par Copyright (c):  RM2020电控
*  
* @par 日志
    2020-4-1    Thunderdoge 添加了周期性限幅函数
*/  
#include "app_math.h"
#include <math.h>

///** 
//* @brief   限幅函数 
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
* @brief   求平方根的倒数
* @remarks 使用经典的Carmack算法，效率高，输入根号下的内容
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

/*****带系数修改的一阶滤波函数

入口：LPF         滤波器结构体
			NEW_DATA    新采样的值
      OLD_DATA    上次滤波获得的结果
      k           滤波系数(代表在滤波结果中的权重)
出口： result      本次滤波结果
 */
float app_math_Lpf1apply(LPF1* LPF,float NEW_DATA,float OLD_DATA,float k)
{
    //角度变化方向，new_flag=1表示角度增加，=0表示角度正在减小
    if((NEW_DATA-OLD_DATA)>0)
        LPF->new_flag=1;
    else if((NEW_DATA-OLD_DATA)<0)
        LPF->new_flag=0;


    if(LPF->new_flag==LPF->last_flag)  //此次变化与前一次变化方向是否一致，相等表示角度变化方向一致
        {
            LPF->num_x++;
            if(fabs((NEW_DATA-OLD_DATA))>Threshold_1)
        //当变化大于Threshold_1的时候，进行计数器num快速增加，以达到快速增大K值，提高跟随性
                LPF->num_x+=5;                           

            if(LPF->num_x>Threshold_2)   //计数阈值设置，当数据递增或递减速度达到一定速率时，增大K值
            {
                LPF->K_x=k+0.2f;          //0.2为K_x的增长值，看实际需要修改
                LPF->num_x=0;
            }
        }
    else 
        {
            LPF->num_x=0;
            LPF->K_x=0.01f;     //数据变化稳定时K_x值，看实际修改
        }
		LPF->last_flag = LPF->new_flag;
    OLD_DATA=(1-LPF->K_x)*OLD_DATA+LPF->K_x*NEW_DATA;
    return OLD_DATA;
}
/** 
* @brief  一阶高通滤波器
* @param[in]   inArgName input argument description. 
* @retval  无
* @par 日志 
*
*/
void app_math_hpf1apply(HPF1* HPF,float sample_freq, float cutoff_freq)
{
}

/** 
* @brief    二阶低通滤波器
* @remarks 
* @par 日志
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









