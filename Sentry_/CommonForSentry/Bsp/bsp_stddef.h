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

#define SET_BIT_N_OF_X(N,X)     (X |= (1U << N))
#define RESET_BIT_N_OF_X(N,X)   (X &= (~(1U << N)))
#define GET_BIT_N_OF_X(N,X)     (((X >> N) & 0x1) == 0X1)

#endif // __BSP_STDDEF_H_
