/**
  * @file  �ڱ���ȫ�ֺ궨��/����ͷ�ļ�
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

// �궨�壺ȡ�������� x �� ĳһλ n ���� n~m λ. n,m ����������0;
#define SET_BIT_NTH(x, n)       ( x |= ((1U)<<(n)) )                    // ��X�ĵ� n λΪ1
#define SET_BIT_N_TO_M(x,n,m)   ( x |= (~((~0u)<<(m-n+1)))<<(n))        // ��X�ĵ� n �� m λΪ1. ���� n>m
#define RESET_BIT_NTH(x, n)     ( x &= ~((1U)<<(n)) )                   // ��X�ĵ� n λΪ0
#define RESET_BIT_N_TO_M(x,n,m) ( x &= ((~0u)<<(m-n+1))<<(n))
#define GET_BIT_NTH(x,n)        ( !( ( x & ( (1U) << n ) ) == 0 ) )     // ȡX�ĵ� n λ
#define GET_BITS(x, n, m)       ( (x & ~(~(0U)<<(m-n+1))<<(n)) >> (n) )


#endif // __BSP_STDDEF_H_
