//VisionUart v0.1, 2019/11/12 by ThunderDoge
#ifndef	__BSP_VISION_UART_H_
#define	__BSP_VISION_UART_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "usart.h"

#define VISION_HUART huart1		//串口句柄
#define VISION_REC_LENGTH 36	//串口缓存数组长度
#define DATA_FRAME_LENGTH 18	//数据帧长度

void bsp_VisionUart_Init(void);	//初始化
void bsp_VisionUart_IT(void);	//中断缓冲处理
HAL_StatusTypeDef bsp_VisionUart_Send(uint8_t* PackedData);	//串口发送
HAL_StatusTypeDef bsp_VisionUart_Receive(uint8_t* RxBuff);	//串口接收

#endif	//__APP_VISION_UART_H_
