/**
 * @file      bsp_tof10120.h
 * @brief     
 * @details   
 * @author   ThunderDoge
 * @date      2020-5-24
 * @version   v0.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: utf-8
 */

#ifndef __BSP_TOF10120_H
#define __BSP_TOF10120_H


#include "usart.h"

//#define USE_MEAN_UPDATE_TIME // 宏定义：计算平均时间刷新

#ifdef USE_MEAN_UPDATE_TIME
#include "app_math.h"
#endif // USE_MEAN_UPDATE_TIME


typedef enum __bsp_tof10120_cmd
{
    tofCmdOffset,
    tofCmdTxGap,
    tofCmdFilter,
    tofCmdMaxDist,
    tofCmdTxMode,
}bsp_tof10120_cmd;

typedef struct 
{
    uint8_t IsInitedFlag;

    uint32_t Distance;
    uint32_t LastUpdateTick;
    #ifdef USE_MEAN_UPDATE_TIME
        float MeanUpdateTime;
        LPF2 UpdateTimeLPF;
    #endif

    int8_t Offset;
    uint16_t TranferGap;
    uint8_t FilterMode;
    uint16_t MaxDistance;
    uint8_t TransferMode;
    uint8_t RxXfer[20];
    uint8_t TxXfer[10];
    uint32_t FailedIdleCnt;

    UART_HandleTypeDef* uart_interface;
}bsp_tof10120_TypeDef;

void bsp_tof10120_Init(bsp_tof10120_TypeDef*,UART_HandleTypeDef*);
void bsp_tof10120_IT(bsp_tof10120_TypeDef*);
HAL_StatusTypeDef bsp_tof10120_ReadParam(bsp_tof10120_TypeDef*,bsp_tof10120_cmd);
HAL_StatusTypeDef bsp_tof10120_WriteParam(bsp_tof10120_TypeDef*,bsp_tof10120_cmd,int32_t cmd_value);

#endif // !__BSP_TOF10120_H

