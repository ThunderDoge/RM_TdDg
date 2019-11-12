/** 
* @brief    小主机通讯库，板级支持版
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
	__HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);//清除中断标志位
	__HAL_UART_ENABLE_IT(&VISION_HUART,UART_IT_IDLE);//使能串口空闲中断
	HAL_UART_Receive_DMA(&VISION_HUART,(uint8_t*)Vision_Rx_Buf_1,VISION_REC_LENGTH);	//开启DMA传输
}

void bsp_VisionUart_IT(void)	//缓冲处理
{
	if(__HAL_UART_GET_FLAG(&VISION_HUART,UART_FLAG_IDLE) != RESET)//空闲中断
	{
		static BaseType_t VisionUart_xHigherPriorityTaskWoken;

		HAL_UART_DMAStop(&VISION_HUART); //关闭DMA
		#ifdef VUART_USE_SEMAPHORE
		xSemaphoreGiveFromISR(VisionUart_Update_Handle,&VisionUart_xHigherPriorityTaskWoken);	//接收完成，给出接收完成信号量
		#else
		VisionUart_Update = 1;
		#endif
		memcpy(Vision_Rx_Buf_2 , Vision_Rx_Buf_1 , VISION_REC_LENGTH );	//复制到2级缓存
		
		__HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART); //清除空闲中断标志位
		HAL_UART_DMAResume(&VISION_HUART);              //重新打开DMA
		HAL_UART_Receive_DMA(&VISION_HUART, (uint8_t*)Vision_Rx_Buf_1, VISION_REC_LENGTH);//重新开启DMA接收传输
	}
}

HAL_StatusTypeDef bsp_VisionUart_Send(uint8_t* PackedData)
{
	return HAL_UART_Transmit(&VISION_HUART , PackedData , DATA_FRAME_LENGTH ,0xff);	//发送数据帧
}

/**
* @brief  串口数据接收到缓存数组
* @detail	使用全局变量 VisionUart_Update 标记是否更新
* @param[in]  RxBuff	缓存数组指针
* @retval  执行结构
*/
HAL_StatusTypeDef bsp_VisionUart_Receive(uint8_t* RxBuff)
{
	#ifdef VUART_USE_SEMAPHORE
	xSemaphoreTake(VisionUart_Update_Handle,portMAX_DELAY);		//等待信号量
	memcpy( RxBuff, Vision_Rx_Buf_2, VISION_REC_LENGTH );	//复制缓存到目标位置
	return HAL_OK;

	#else
	if(VisionUart_Update == 1){
		memcpy( RxBuff, Vision_Rx_Buf_2, VISION_REC_LENGTH );	//复制缓存到目标位置
		VisionUart_Update = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
	#endif	
}

