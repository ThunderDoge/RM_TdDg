/**
  * @file      app_judge_uart.c
  * @brief     裁判系统学生串口接收处理
  * @details   
  * @author   ThunderDoge
  * @date      2020-4-9
  * @version   v0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */
#include "app_judge_uart.h"

uint8_t app_judge_framehead_check(uint8_t* head_ptr);
uint8_t app_judge_frametail_check(uint8_t* tail_ptr);
uint16_t app_judge_frame_cmdid_data_analyze(uint8_t* after_head_ptr);
uint8_t calc_crc8(uint8_t data, uint16_t data_len);
uint8_t calc_crc16(uint8_t data, uint16_t data_len);


void app_judge_AnalysisXfer(uint8_t* Xfer)
{
    
}
