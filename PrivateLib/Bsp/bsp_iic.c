/** 
* @brief    IIC�弶֧�ְ�
* @details  IICЭ�����ʱ����
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/

#include "bsp_iic.h"

/** 
* @brief  IIC��ʱ
* @details  IIC����ʱ����ʱ����
* @param  NULL
* @retval  NULL
*/
static void bsp_IIC_Delay(void)
{
	/*��ѧ�����ʱ����ͷû��ʾ�����ⲻ�ˣ��ò��˵ľ͸���ѭ������*/
	uint8_t i;
	for (i = 0; i < 200; i++);
}

/**
* @brief  IIC��ʼ��
* @details  IIC���ų�ʼ����ɺ���ֹͣ�ź�
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_Init(void)
{
	bsp_IIC_Stop();	/*��ʼ����ֹͣ����*/
}

/**
* @brief  IIC��������
* @details  ����IIC���������ź�
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_Start(void)
{
	Bsp_IIC_SCL_HIGH;
	Bsp_IIC_SDA_HIGH;
	bsp_IIC_Delay();
	BSP_IIC_SDA_LOW;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_LOW;
	bsp_IIC_Delay();
}

/**
* @brief  IICֹͣ
* @details  ����IIC����ֹͣ�ź�
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_Stop(void)
{
	BSP_IIC_SDA_LOW;
	Bsp_IIC_SCL_LOW;
	bsp_IIC_Delay();
	Bsp_IIC_SDA_HIGH;
}

/**
* @brief  IICӦ��
* @details  ���߲���һ��ACK�ź�
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_Ack(void)
{
	BSP_IIC_SDA_LOW;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_HIGH;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_LOW;
	bsp_IIC_Delay();
	Bsp_IIC_SDA_HIGH;
}

/**
* @brief  IIC��Ӧ��
* @details  IIC���߲���һ��NAck�ź�
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_NAck(void)
{
	Bsp_IIC_SDA_HIGH;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_HIGH;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_LOW;
	bsp_IIC_Delay();
}

/**
* @brief  IIC�ȴ�Ӧ��
* @details  IIC���ߵȴ��ӻ�Ӧ���ź�
* @param  NULL
* @retval  1Ӧ�� 0��Ӧ��
*/
uint8_t bsp_IIC_WaitAck(void)
{
	uint8_t State;
	
	Bsp_IIC_SDA_HIGH;
	bsp_IIC_Delay();
	Bsp_IIC_SCL_HIGH;
	bsp_IIC_Delay();
	
	if(Bsp_IIC_SDA_READ)
	{
		State = 1;
	}
	else
	{
		State = 0;
	}
	Bsp_IIC_SCL_LOW;
	bsp_IIC_Delay();
	
	return State;
}

/**
* @brief  IIC��������
* @details  IIC���߷���һ���ֽ�����
* @param  uint8_t message �����͵���Ϣ
* @retval  NULL
*/
void bsp_IIC_SendByte(uint8_t message)
{
	int i;
	for(i=0; i<8; i++)
	{
		if(message & 0x80)
		{
			Bsp_IIC_SDA_HIGH;
		}
		else
		{
			BSP_IIC_SDA_LOW;
		}
		bsp_IIC_Delay();
		Bsp_IIC_SCL_HIGH;
		bsp_IIC_Delay();
		Bsp_IIC_SCL_LOW;
		if(i == 7)
		{
			Bsp_IIC_SDA_HIGH;
		}
		message <<= 1;
		bsp_IIC_Delay();
	}
}

/**
* @brief  IIC��ȡ����
* @details  IIC���߶�ȡһ���ֽ�����
* @param  uint8_t Ack �ȴ���ȡ��IIC�ӻ�
* @retval  value ��ȡ������
*/
uint8_t bsp_IIC_ReadByte(uint8_t Ack)
{
	int i;
	uint8_t value;
	
	for(i=0; i<8; i++)
	{
		value <<= 1;
		Bsp_IIC_SCL_HIGH;
		bsp_IIC_Delay();
		if(Bsp_IIC_SDA_READ)
		{
			value++;
		}
		Bsp_IIC_SCL_LOW;
		bsp_IIC_Delay();
	}
	if(Ack == 0)
	{
		bsp_IIC_NAck();
	}
	else
	{
		bsp_IIC_Ack();
	}
	return value;
}

