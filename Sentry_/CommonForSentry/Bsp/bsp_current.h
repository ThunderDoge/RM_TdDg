#ifndef __BSP_CURRENT_H
#define __BSP_CURRENT_H

#include "main.h"
#include "cmsis_os.h"


extern int16_t bsp_CurrentRead[5];
extern int16_t bsp_VoltageRead[5];

void bsp_Current_Init(void);
void bsp_Current_Read(void);
void bsp_Current_StartRead_IT(uint8_t ID);

void bsp_Current_Read_IT_RxCplt(uint8_t ID);


#endif 
