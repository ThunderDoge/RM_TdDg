/**
  * @file  哨兵用全局宏定义/常量头文件
  * @brief    
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/24
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __BSP_STDDEF_H_
#define __BSP_STDDEF_H_

#include "stm32f4xx_hal.h"

#include <stdio.h>


// define user fputc() implement for printf()
#ifdef __stdio_h

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	if(HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,0xff) == HAL_OK)
		return ch;
	else
		return EOF;
}

#undef PUTCHAR_PROTOTYPE

#endif //__stdio_h

#endif // __BSP_STDDEF_H_
