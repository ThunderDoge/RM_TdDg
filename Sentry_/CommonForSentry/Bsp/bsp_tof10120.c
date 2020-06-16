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
#include "bsp_tof10120.h"
#include "stdio.h"
#include "string.h"

HAL_StatusTypeDef bsp_bsp_tof10120_RxXferAnalysis(bsp_tof10120_TypeDef* module);

void bsp_tof10120_Init(bsp_tof10120_TypeDef* module, UART_HandleTypeDef* uart)
{
    module->uart_interface  =uart;
    module->IsInitedFlag = 1;

    uart->Init.BaudRate = 9600;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    
    if (HAL_UART_Init(uart) != HAL_OK)
    {
        Error_Handler();
    }

    #ifdef USE_MEAN_UPDATE_TIME
    app_math_LPF2pSetCutoffFreq(&module->UpdateTimeLPF,100,10);
    #endif
	
	__HAL_UART_ENABLE_IT(module->uart_interface,UART_IT_IDLE);


    bsp_tof10120_WriteParam(module, tofCmdTxGap,10);

    HAL_UART_Receive_IT(uart,module->RxXfer,sizeof(module->RxXfer)-1);

    bsp_tof10120_ReadParam(module, tofCmdTxGap);
}

void bsp_tof10120_IT(bsp_tof10120_TypeDef * module)
{
    if(!module->IsInitedFlag)
    {   Error_Handler();    }

    if(__HAL_UART_GET_FLAG(module->uart_interface,UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(module->uart_interface);
        HAL_UART_AbortReceive_IT(module->uart_interface);

        if(bsp_bsp_tof10120_RxXferAnalysis(module) != HAL_OK)
            module->FailedIdleCnt ++;

        HAL_UART_Receive_IT(module->uart_interface,module->RxXfer,sizeof(module->RxXfer)-1);
    }
}
/**
 * @brief 发送改写模块参数的命令
 * 
 * @param module 被操作的模块的指针
 * @param cmd 命令字，取值见 @see bsp_tof10120_cmd 枚举型
 *      @arg tofCmdOffset : 偏移量，取值+/- 0~99
 *      @arg tofCmdTxGap  : 发送间隔，取值10~2000
 *      @arg tofCmdFilter : 滤波模式，0=滤波后，1=不滤波
 *      @arg tofCmdMaxDist: 设置最大距离，取值0~2000，默认2000
 *      @arg tofCmdTxMode : 发送方式，0=主动发送，1=被动发送
 * @param cmd_value 命令值。没写取值合法检查assertParam，请自行注意参数取值正确
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef bsp_tof10120_WriteParam(bsp_tof10120_TypeDef * module , bsp_tof10120_cmd cmd , int32_t cmd_value)
{
    int add_i,tx_cnt;
    module->TxXfer[0] = 's';
    module->TxXfer[1] = '0'+((uint8_t)cmd)+1;

    if(cmd == tofCmdOffset)
    {
        if(cmd_value >= 0)
        {
            module->TxXfer[2] = '+';
            add_i = sprintf((char*)&module->TxXfer[3],"%d",cmd_value);
            module->TxXfer[3+add_i] = '#';
            tx_cnt = 4+add_i;
        }
        else
        {
            add_i = sprintf((char*)&module->TxXfer[3],"%d",cmd_value);
            module->TxXfer[3+add_i] = '#';
            tx_cnt = 4+add_i;
        }
    }
    else
    {
        module->TxXfer[2] = '-';
        add_i = sprintf((char*)&module->TxXfer[3],"%d",cmd_value);
        module->TxXfer[3+add_i] = '#';
        tx_cnt = 4+add_i;
    }
    return HAL_UART_Transmit(module->uart_interface,module->TxXfer,tx_cnt,1U);
}
/**
 * @brief 发送读取模块参数的命令
 * 
 * @param module 被操作的模块的指针
 * @param cmd 命令字，取值见 @see bsp_tof10120_cmd 枚举型
 *      @arg tofCmdOffset : 偏移量，取值+/- 0~99
 *      @arg tofCmdTxGap  : 发送间隔，取值10~2000
 *      @arg tofCmdFilter : 滤波模式，0=滤波后，1=不滤波
 *      @arg tofCmdMaxDist: 设置最大距离，取值0~2000，默认2000
 *      @arg tofCmdTxMode : 发送方式，0=主动发送，1=被动发送
 * @return HAL_StatusTypeDef 
 */

HAL_StatusTypeDef bsp_tof10120_ReadParam(bsp_tof10120_TypeDef * module, bsp_tof10120_cmd cmd)
{
    module->TxXfer[0] = 'r';
    module->TxXfer[1] = '0'+((uint8_t)cmd)+1;
    module->TxXfer[2] = '#';
    return HAL_UART_Transmit(module->uart_interface,module->TxXfer,3,1U);
}

HAL_StatusTypeDef bsp_bsp_tof10120_RxXferAnalysis(bsp_tof10120_TypeDef* module)
{
    int ok = 0;

    module->RxXfer[sizeof(module->RxXfer)-1] = '\0';

    if(strstr((char*)module->RxXfer,"mm")!=NULL)
    {
        sscanf((char*)module->RxXfer,"%dmm",&module->Distance);
        ok = 1;
    }
    else if(strstr((char*)module->RxXfer,"D"))
    {
        sscanf((char*)module->RxXfer,"D=%d",&module->Offset);
        ok = 1;
    }
    else if(strstr((char*)module->RxXfer,"T"))
    {
        sscanf((char*)module->RxXfer,"T=%d",&module->TranferGap);
        ok = 1;
    }
    else if(strstr((char*)module->RxXfer,"M"))
    {
        sscanf((char*)module->RxXfer,"M=%d",&module->FilterMode);
        ok = 1;
    }
    else if(strstr((char*)module->RxXfer,"Max"))
    {
        sscanf((char*)module->RxXfer,"Max=%d",&module->MaxDistance);
        ok = 1;
    }
    else if(strstr((char*)module->RxXfer,"S"))
    {
        sscanf((char*)module->RxXfer,"S=%d",&module->TransferMode);
        ok = 1;
    }

    memset(module->RxXfer,0,sizeof(module->RxXfer));

    if(ok)
    {
        #ifdef USE_MEAN_UPDATE_TIME
            module->MeanUpdateTime = app_math_LPF2pApply(&module->UpdateTimeLPF, (HAL_GetTick() - module->LastUpdateTick) );
        #endif
        module->LastUpdateTick = HAL_GetTick();
        return HAL_OK;
    }
    else
        return HAL_ERROR;
}