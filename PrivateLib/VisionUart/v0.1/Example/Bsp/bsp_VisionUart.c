/** 
* @brief    С����ͨѶ�⣬�弶֧�ְ�
* @details  
* @author   Thunderdoge
* @date      2019/11/12
* @version  v0.1
* @par Copyright (c):  OnePointFive, UESTC, 2019~2020
	OnePointFIve, the UESTC RoboMaster Team.
*/

#include "bsp_VisionUart.h"

#ifdef VUART_USE_SEMAPHORE
xSemaphoreHandle VisionUart_Update_Handle;
#else
static uint8_t VisionUart_Update;
#endif
uint8_t Vision_Rx_Buf_1[VISION_REC_LENGTH];
uint8_t Vision_Rx_Buf_2[VISION_REC_LENGTH];

void bsp_VisionUart_Init(void)
{
	#ifdef VUART_USE_SEMAPHORE
	vSemaphoreCreateBinary(VisionUart_Update_Handle);
	#endif
	__HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);//����жϱ�־λ
	__HAL_UART_ENABLE_IT(&VISION_HUART,UART_IT_IDLE);//ʹ�ܴ��ڿ����ж�
	HAL_UART_Receive_DMA(&VISION_HUART,(uint8_t*)Vision_Rx_Buf_1,VISION_REC_LENGTH);	//����DMA����
}

void bsp_VisionUart_IT(void)	//���崦��
{
	if(__HAL_UART_GET_FLAG(&VISION_HUART,UART_FLAG_IDLE) != RESET)//�����ж�
	{
		static BaseType_t VisionUart_xHigherPriorityTaskWoken;

		HAL_UART_DMAStop(&VISION_HUART); //�ر�DMA
		#ifdef VUART_USE_SEMAPHORE
		xSemaphoreGiveFromISR(VisionUart_Update_Handle,&VisionUart_xHigherPriorityTaskWoken);	//������ɣ�������������ź���
		#else
		VisionUart_Update = 1;
		#endif
		memcpy(Vision_Rx_Buf_2 , Vision_Rx_Buf_1 , VISION_REC_LENGTH );	//���Ƶ�2������
		
		__HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART); //��������жϱ�־λ
		HAL_UART_DMAResume(&VISION_HUART);              //���´�DMA
		HAL_UART_Receive_DMA(&VISION_HUART, (uint8_t*)Vision_Rx_Buf_1, VISION_REC_LENGTH);//���¿���DMA���մ���
	}
}

HAL_StatusTypeDef bsp_VisionUart_Send(uint8_t* PackedData)
{
	return HAL_UART_Transmit(&VISION_HUART , PackedData , DATA_FRAME_LENGTH ,0xff);	//��������֡
}

/**
* @brief  �������ݽ��յ���������
* @detail	ʹ��ȫ�ֱ��� VisionUart_Update ����Ƿ����
* @param[in]  RxBuff	��������ָ��
* @retval  ִ�нṹ
*/
HAL_StatusTypeDef bsp_VisionUart_Receive(uint8_t* RxBuff)
{
	#ifdef VUART_USE_SEMAPHORE
	xSemaphoreTake(VisionUart_Update_Handle,portMAX_DELAY);		//�ȴ��ź���
	memcpy( RxBuff, Vision_Rx_Buf_2, VISION_REC_LENGTH );	//���ƻ��浽Ŀ��λ��
	return HAL_OK;

	#else
	if(VisionUart_Update == 1){
		memcpy( RxBuff, Vision_Rx_Buf_2, VISION_REC_LENGTH );	//���ƻ��浽Ŀ��λ��
		VisionUart_Update = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
	#endif	
}

