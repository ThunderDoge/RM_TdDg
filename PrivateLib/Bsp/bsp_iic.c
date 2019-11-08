/** 
* @brief    IIC板级支持包
* @details  IIC协议相关时序函数
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/

#include "bsp_iic.h"

/** 
* @brief  IIC延时
* @details  IIC总线时序延时函数
* @param  NULL
* @retval  NULL
*/
static void bsp_IIC_Delay(void)
{
	/*玄学软件延时，手头没有示波器测不了，用不了的就改下循环次数*/
	uint8_t i;
	for (i = 0; i < 200; i++);
}

/**
* @brief  IIC初始化
* @details  IIC引脚初始化完成后发送停止信号
* @param  NULL
* @retval  NULL
*/
void bsp_IIC_Init(void)
{
	bsp_IIC_Stop();	/*初始化，停止总线*/
}

/**
* @brief  IIC总线启动
* @details  发送IIC总线启动信号
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
* @brief  IIC停止
* @details  发送IIC总线停止信号
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
* @brief  IIC应答
* @details  总线产生一个ACK信号
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
* @brief  IIC非应答
* @details  IIC总线产生一个NAck信号
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
* @brief  IIC等待应答
* @details  IIC总线等待从机应答信号
* @param  NULL
* @retval  1应答 0无应答
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
* @brief  IIC发送数据
* @details  IIC总线发送一个字节数据
* @param  uint8_t message 待发送的信息
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
* @brief  IIC读取数据
* @details  IIC总线读取一个字节数据
* @param  uint8_t Ack 等待读取的IIC从机
* @retval  value 读取的数据
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

