#ifndef _FILTER_H
#define _FILTER_H

#ifdef  STM32F405xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#else
#include "stm32f1xx_hal.h"
#endif
#include <math.h>

#define 	PI 		3.1415926535f
#define SECTOR_11								FLASH_SECTOR_11
#define SECTOR_11_STR_ADDR			(uint32_t)(0x080E0000)	
#define USER_FLASH_END_ADDRESS	(uint32_t)(0x080FFFFF)

typedef struct  //< butterworth lowpass filter 
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float G;   //�������
    
    float current_input;		     
    float last_input;			       
    float pre_input;
    
    float current_output;		            //y[n]
    float last_output;			            //y[n-1]
    float pre_output;	                  //y[n-2]
}IIR;

typedef struct 
{
    float P_last;				/*�ϴ�Ԥ�����Э�������P(k|k-1)*/
    float X_last;				/*ϵͳ״̬Ԥ�����X(k|k-1)���о���*/
    
    float Q;						/*���̰�����Э����ϵ��*/
    float R;						/*�۲������Э����ϵ��*/
    
    float K;						/*����������K(k)���о���*/
    float X;						/*���Ź���״̬��������X(k|k)���о���*/
    float P;						/*���Ź���Э�������P(k|k)*/
                                                        
    float input;				    /*ϵͳ���ֵ����Z(k)*/
    uint8_t flag;
}kalman_filter;	

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
}LPF2;

void LPF2pSetCutoffFreq(LPF2* LPF,float sample_freq, float cutoff_freq);
float LPF2pApply(LPF2* LPF,float sample);
float Kalman(kalman_filter* kalman,float input);

uint8_t Flash_WriteIn(uint8_t destination,int16_t* pData,uint8_t len);
int16_t Flash_ReadOut(uint8_t destination);
#endif

