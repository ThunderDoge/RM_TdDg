#include "task_VisionUart.h"

void TaskStarter(void)
{
	bsp_VisionUart_Init();
	xTaskCreate(task_VisionUart_Rx,"task_VisionUart_Rx",512,NULL,4,NULL);
	xTaskCreate(task_VisionUart_TxTest,"task_VisionUart_TxTest",512,NULL,4,NULL);
}

void task_VisionUart_Rx(void* param)
{
	while(1)
	{
		app_VisionUart_RecvPack();
		app_VisionUart_AckMsg();
	}
	//Task shoule never run here.
	vTaskDelete(NULL);	
}
void task_VisionUart_TxTest(void* param)
{
	while(1)
	{
		app_VisionUart_TxTest(1000);
		vTaskDelay(3000);
	}
	//Task shoule never run here.
	vTaskDelete(NULL);
}



