/** 
 * @file    bsp_dbus.c
* @brief    Dbus�弶֧�ְ�
* @details  Dbus���ݽ��գ�����
* @author   Evan-GH
* @date      2019.10
* @version  1.3
* @par Copyright (c):  RM2020���
* @par 	����ʹ�÷�����Readme.md
				�汾���:
				1.0		|		��ͨDbus��������жϴ�����,���ݽ�������
				1.1		|		��ӶԽ������ݵ���Ч�Ժ���ȷ�Ե�У��
				1.2		|		������������,��Ϊʹ���ź�������ͨ�ж�����
				1.3		|		���ӶԽ��ջ������鴦��֮�����0������ȷ������У��׼ȷ
				1.4		|		���ݴ���淶�޸���һЩ��������
				1.5		|		�޸�һЩСbug
				2.0		2020-3-18	ThunderDoge		���ϵ��ڱ������С����������߼�ⲿ�֡�
*/
#include "bsp_dbus.h"

/// ���߼�� �ṹ��
CheckDevice_Type Dbus_CheckDevice(DbusDevice,100);


bsp_dbus_RC_Data bsp_dbus_Data; //Dbus��������
static uint8_t Dbus_Rxbuffer[BSP_DBUS_BUFFER_SIZE]={0}; //Dbus�������ݻ�������
int16_t Dbus_CHx_StaticOffset[4]={0};	//ң������������״̬ʱ��ƫ�ơ�Ĭ��Ϊ0.�޸Ĵ�ֵ���ý��������bsp_dbus_Dataֵ������

/**
* @brief  Dbus���߳�ʼ��
* @details  ��ʼ��Dbus�ж����ã�����DMA���տ����ж�
* @param  NULL
* @retval  NULL
*/
void bsp_dbus_Init(void)
{
	// ���߼��ṹ�� ����
	app_sentry_CheckDevice_AddToArray(&Dbus_CheckDevice);
																
	
	__HAL_UART_CLEAR_IDLEFLAG(&BSP_DBUS_UART); //��������ж�λ
	__HAL_UART_ENABLE_IT(&BSP_DBUS_UART,UART_IT_IDLE); //ʹ��DMA���տ����ж�
	HAL_UART_Receive_DMA(&BSP_DBUS_UART,Dbus_Rxbuffer,BSP_DBUS_BUFFER_SIZE); //��ʼDMA����
}

/**
* @brief  Dbus��������У��
* @details  У��������ݵ���Ч�Ժ���ȷ��
* @param  NULL
* @retval  HAL_OK ��������  HAL_ERROR ��������
*/
static HAL_StatusTypeDef bsp_dbus_Datacheck(void)
{
	int i=0;
	for(i=0;i<18;i++)				//���ȼ���ǰ��18λ�����Ƿ���Ч�����ȫΪ0��Ȼ����Ч����
	{
		if(Dbus_Rxbuffer[i]==0)
		{
			continue;					
		}
		else
		{
			break;
		}
	}
	if(i==18)
	{
		return HAL_ERROR;				//ǰ18λ����ȫ��Ϊ0�����ش���
	}
	for(i=18;i<25;i++)											//�Ժ�������ݽ��м��飬�����Ϊ0��Ȼ���ݳ���
	{
		if(Dbus_Rxbuffer[i]!=0)
		{
			return HAL_ERROR;			//���漸λ���ݳ����˷�0��˵�����ݳ���
		}
	}
	return HAL_OK;
}

/**
* @brief  Dbus���ݽ���
* @details  ����Dbus���յ������ݲ����µ��ṹ�壬����һ��
* @param  NULL
* @retval  NULL
*/
#ifdef BSP_DBUS_USE_SIGNAL
	void bsp_dbus_Analysis(void)
	{
		// ң������ԭʼ�������ⲿ�ִ���ֱ�Ӳο��ٷ�ʾ�����뼴��
		//ң�������Ʋ���
		bsp_dbus_Data.CH_0 = ((int16_t)Dbus_Rxbuffer[0] | (((int16_t)Dbus_Rxbuffer[1]<<8))) & 0x07FF;
		bsp_dbus_Data.CH_1 = ((int16_t)Dbus_Rxbuffer[1] >> 3 | (((int16_t)Dbus_Rxbuffer[2]<<5))) & 0x07FF;
		bsp_dbus_Data.CH_2 = ((int16_t)Dbus_Rxbuffer[2] >> 6 | (((int16_t)Dbus_Rxbuffer[3]<<2)) | ((int16_t)Dbus_Rxbuffer[4])<<10) & 0x07FF;
		bsp_dbus_Data.CH_3 = ((int16_t)Dbus_Rxbuffer[4] >> 1 | (((int16_t)Dbus_Rxbuffer[5]<<7)) ) & 0x07FF;
		bsp_dbus_Data.S1 = ((Dbus_Rxbuffer[5] >> 4) & 0X000C) >> 2;
		bsp_dbus_Data.S2 = (Dbus_Rxbuffer[5] >> 4) & 0X0003;			
		bsp_dbus_Data.Dial = ((int16_t)Dbus_Rxbuffer[16] | ((int16_t)Dbus_Rxbuffer[17] << 8)) & 0x07FF;		//���������ݣ���Ҫ����ң�����̼�
		
		//������Ʋ���
		bsp_dbus_Data.Mouse.X = ((int16_t)Dbus_Rxbuffer[6]) | ((int16_t)Dbus_Rxbuffer[7] << 8);
		bsp_dbus_Data.Mouse.Y = ((int16_t)Dbus_Rxbuffer[8]) | ((int16_t)Dbus_Rxbuffer[9] << 8);
		bsp_dbus_Data.Mouse.Z = ((int16_t)Dbus_Rxbuffer[10]) | ((int16_t)Dbus_Rxbuffer[11] << 8);
		bsp_dbus_Data.Mouse.Leftkey = Dbus_Rxbuffer[12];
		bsp_dbus_Data.Mouse.Rightkey = Dbus_Rxbuffer[13];
		bsp_dbus_Data.Keys = ((int16_t)Dbus_Rxbuffer[14] | (int16_t)Dbus_Rxbuffer[15]<<8);
	}
#else
	static void bsp_dbus_Analysis(void)
	{
		// ң������ԭʼ�������ⲿ�ִ���ֱ�Ӳο��ٷ�ʾ�����뼴��
		//ң�������Ʋ���
		bsp_dbus_Data.CH_0 = ((int16_t)Dbus_Rxbuffer[0] | (((int16_t)Dbus_Rxbuffer[1]<<8))) & 0x07FF;
		bsp_dbus_Data.CH_1 = ((int16_t)Dbus_Rxbuffer[1] >> 3 | (((int16_t)Dbus_Rxbuffer[2]<<5))) & 0x07FF;
		bsp_dbus_Data.CH_2 = ((int16_t)Dbus_Rxbuffer[2] >> 6 | (((int16_t)Dbus_Rxbuffer[3]<<2)) | ((int16_t)Dbus_Rxbuffer[4])<<10) & 0x07FF;
		bsp_dbus_Data.CH_3 = ((int16_t)Dbus_Rxbuffer[4] >> 1 | (((int16_t)Dbus_Rxbuffer[5]<<7)) ) & 0x07FF;
		bsp_dbus_Data.S1 = ((Dbus_Rxbuffer[5] >> 4) & 0X000C) >> 2;
		bsp_dbus_Data.S2 = (Dbus_Rxbuffer[5] >> 4) & 0X0003;			
		bsp_dbus_Data.Dial = ((int16_t)Dbus_Rxbuffer[16] | ((int16_t)Dbus_Rxbuffer[17] << 8)) & 0x07FF;		//���������ݣ���Ҫ����ң�����̼�
		
		//������Ʋ���
		bsp_dbus_Data.Mouse.X = ((int16_t)Dbus_Rxbuffer[6]) | ((int16_t)Dbus_Rxbuffer[7] << 8);
		bsp_dbus_Data.Mouse.Y = ((int16_t)Dbus_Rxbuffer[8]) | ((int16_t)Dbus_Rxbuffer[9] << 8);
		bsp_dbus_Data.Mouse.Z = ((int16_t)Dbus_Rxbuffer[10]) | ((int16_t)Dbus_Rxbuffer[11] << 8);
		bsp_dbus_Data.Mouse.Leftkey = Dbus_Rxbuffer[12];
		bsp_dbus_Data.Mouse.Rightkey = Dbus_Rxbuffer[13];
		bsp_dbus_Data.Keys = ((int16_t)Dbus_Rxbuffer[14] | (int16_t)Dbus_Rxbuffer[15]<<8);
		
		//�Զ��崦��By ThunderDoge 2019/11/23
		bsp_dbus_Data.CH_0 -= 1024+Dbus_CHx_StaticOffset[0];
		bsp_dbus_Data.CH_1 -= 1024+Dbus_CHx_StaticOffset[1];
		bsp_dbus_Data.CH_2 -= 1024+Dbus_CHx_StaticOffset[2];
		bsp_dbus_Data.CH_3 -= 1024+Dbus_CHx_StaticOffset[3];
		
		//���߼�����
		Dbus_CheckDevice.update_hook_func(&Dbus_CheckDevice);
	}
#endif

/**
* @brief  Dbus�жϴ�����
* @details  �ͷŶ�ֵ�ź����Թ��������ͬ����������Dbus��ʹ�õĴ��ڵ������ж��е��� ע���������Ч�Ժ���ȷ�Խ��м��
* @param  NULL
* @retval  NULL
*/
void bsp_dbus_It(void)
{
	if(__HAL_UART_GET_FLAG(&BSP_DBUS_UART,UART_FLAG_IDLE) != RESET)	//��������˿����ж�
	{		
		HAL_UART_DMAStop(&BSP_DBUS_UART); //�ر�DMA
		//�ڽ�������֮ǰ����ȷ�����ݵ���Ч�Ժ�׼ȷ��
		if(bsp_dbus_Datacheck() == HAL_OK)
		{			
			#ifdef BSP_DBUS_USE_SIGNAL		//���ʹ���ź���
				static BaseType_t Dbus_xHigherPriorityTaskWoken;
				xSemaphoreGiveFromISR(Dbus_Update_Handle,&Dbus_xHigherPriorityTaskWoken); //�ͷ��ź���������Dbus����
				xSemaphoreGiveFromISR(Dbus_Check_Handle, &Dbus_xHigherPriorityTaskWoken);	//�ͷ�Dbus���߼���ֵ�ź���
			#else													//û��ʹ���ź���
				bsp_dbus_Analysis(); //ֱ�����ж������������
			#endif
		}
		memset(Dbus_Rxbuffer,0,BSP_DBUS_BUFFER_SIZE);	//�Խ��ջ���������0
		//���Ե�ʱ�����ȡ���������ע�Ͱ����ݴ�ӡ������أ�ע��printf�������ͺ��������ܵ���ϵͳ���������ԷŽ������Ժ����д�ӡ����
		//ע�⣺ʹ����������ǰ�������Ѿ��ض�����printf��������ȷ֪���ĸ������������
		//printf("CH0:%d CH1:%d CH2:%d CH3:%d\n",bsp_dbus_Data.CH_0,bsp_dbus_Data.CH_1,bsp_dbus_Data.CH_2,bsp_dbus_Data.CH_3);
		//printf("S1:%d S2:%d\n",bsp_dbus_Data.S1,bsp_dbus_Data.S2);
		__HAL_UART_CLEAR_IDLEFLAG(&BSP_DBUS_UART); //��������жϱ�־λ
		HAL_UART_Receive_DMA(&BSP_DBUS_UART, (uint8_t*)Dbus_Rxbuffer, BSP_DBUS_BUFFER_SIZE);//���¿���DMA���մ���
	}
}
