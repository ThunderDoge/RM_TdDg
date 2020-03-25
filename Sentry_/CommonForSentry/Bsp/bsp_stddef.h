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

// 宏定义：取整形数据 x 的 某一位 n 或者 n~m 位. n,m 计数均起于0;
#define SET_BIT_NTH(x, n)       ( x |= ((1U)<<(n)) )                    // 设X的第 n 位为1
#define SET_BIT_N_TO_M(x,n,m)   ( x |= (~((~0u)<<(m-n+1)))<<(n))        // 设X的第 n 到 m 位为1. 必须 n>m
#define RESET_BIT_NTH(x, n)     ( x &= ~((1U)<<(n)) )                   // 设X的第 n 位为0
#define RESET_BIT_N_TO_M(x,n,m) ( x &= ((~0u)<<(m-n+1))<<(n))
#define GET_BIT_NTH(x,n)        ( !( ( x & ( (1U) << n ) ) == 0 ) )     // 取X的第 n 位
#define GET_BITS(x, n, m)       ( (x & ~(~(0U)<<(m-n+1))<<(n)) >> (n) )


#endif // __BSP_STDDEF_H_
