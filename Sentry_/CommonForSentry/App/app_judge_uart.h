/**
  * @file      app_judge_uart.h
  * @brief     裁判系统学生串口接收处理
  * @details   
  * @author   ThunderDoge
  * @date      2020-4-9
  * @version   v0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */

#ifndef __APP_JUDGE_UART_H
#define __APP_JUDGE_UART_H

#include "usart.h"

#include "app_judge_uart_datastruct.h"
#include "app_judge_cmd_id.h"


void app_judge_Init(void);
void app_judge_RxCpltCallback(void);
void app_judge_AnalysisXfer(uint8_t* Xfer);

#endif // __APP_JUDGE_UART_H
