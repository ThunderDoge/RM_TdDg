#ifndef __BSP_CURRENT_H
#define __BSP_CURRENT_H

#include "main.h"


extern int16_t bsp_CurrentRead[5];
extern int16_t bsp_VoltageRead[5];

void bsp_Current_Init(void);
void bsp_Current_Read(void);


#endif 
