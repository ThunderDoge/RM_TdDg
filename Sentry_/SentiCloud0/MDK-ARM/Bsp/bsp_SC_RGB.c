/**
  * @file  bsp_SC_RGB.c
  * @brief    哨兵自定义(SentryCustomized)RGB控制模块
  * @details  
  * @author   ThunderDoge
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "bsp_SC_RGB.h"

#ifdef _USE_SOFTWARE_TIMER 

uint32_t softTimerPeriod;
uint32_t rValue,gValue,bValue;

void bsp_SC_SoftRGB_Task(void* param);


void bsp_SC_SoftRGB_Init(uint32_t period)
{
    softTimerPeriod = period;
	xTaskCreate(bsp_SC_SoftRGB_Task, "bsp_SC_SoftRGB_Task",512,NULL,1,NULL);
}

void bsp_SC_SoftRGB_Task(void* param)
{
	while(1)
	{
		if(rValue>0){
			HAL_GPIO_WritePin(R_GPIO_PROT,R_GPIO_PIN,GPIO_PIN_SET);
			vTaskDelay(rValue);
		}
		if(softTimerPeriod>rValue){
			HAL_GPIO_WritePin(R_GPIO_PROT,R_GPIO_PIN,GPIO_PIN_RESET);
			vTaskDelay(softTimerPeriod-rValue);    
		}
	}
}
void bsp_SC_SoftRGB_Set(uint32_t r, uint32_t g, uint32_t b)
{
	if(r>softTimerPeriod)	r = softTimerPeriod;
	if(g>softTimerPeriod)	g = softTimerPeriod;
	if(b>softTimerPeriod)	b = softTimerPeriod;
	rValue =r;
	gValue =g;
	bValue =b;
}
#endif
