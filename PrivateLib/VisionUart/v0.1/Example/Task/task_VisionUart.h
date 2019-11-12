#ifndef	__TASK_VISION_UART_H_
#define	__TASK_VISION_UART_H_

#include "bsp_VisionUart.h"
#include "app_VisionUart.h"

void task_VisionUart_Rx(void* param);
void task_VisionUart_Ack(void* param);
void task_VisionUart_TxTest(void* param);


void TaskStarter(void);

#endif	//__TASK_VISION_UART_H
