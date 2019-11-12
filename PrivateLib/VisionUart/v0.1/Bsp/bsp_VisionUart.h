//VisionUart v0.1, 2019/11/12 by ThunderDoge
#ifndef	__BSP_VISION_UART_H_
#define	__BSP_VISION_UART_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "usart.h"

#define VISION_HUART huart1		//���ھ��
#define VISION_REC_LENGTH 36	//���ڻ������鳤��
#define DATA_FRAME_LENGTH 18	//����֡����

void bsp_VisionUart_Init(void);	//��ʼ��
void bsp_VisionUart_IT(void);	//�жϻ��崦��
HAL_StatusTypeDef bsp_VisionUart_Send(uint8_t* PackedData);	//���ڷ���
HAL_StatusTypeDef bsp_VisionUart_Receive(uint8_t* RxBuff);	//���ڽ���

#endif	//__APP_VISION_UART_H_
