/**
	****************************(C) COPYRIGHT KisonHe****************************
	* @file       bsp_current.cpp
	* @brief      KisonHe 的电流采样
	* @note
	* @history
	*  Version    Date            Author          Modification
	*  V1.0.0     2019-11-26      KisonHe		  		1. 
	*  V1.5.0	  	2019-12-31	  	KisonHe		  		1.可选的采样平均次数
	*											  											2.所有的电压与电流乘子系数可变
	*											  											3.简化逻辑
	*  V1.5.1	  	2020-1-3	  		KisonHe		  		1.读取总电流
	*  V1.6.0		2020-05-26			KisonHe				允许改变转换时间
	*
	**************************(C) COPYRIGHT KisonHe****************************
	*/
/*
D15		D14		D13		D12		D11		D10		D9		D8		D7		D6		D5		D4		D3		D2		D1		D0
RST 	—		—		—		AVG2	AVG1	AVG0	VBUSCT2 VBUSCT1 VBUSCT0 VSHCT2 VSHCT1 VSHCT0	MODE3	MODE2	MODE1
0		1		0		0
*/
#include "bsp_current.h"
#include "main.h"
#include "i2c.h"

#include <stdlib.h>
#include <string.h>

#define INA226I2C (hi2c2)
#define NUMOFAVG 4   //芯片里面平均的次数，可以取1024，512，256，128，64，16，4，1
#define TIMEOFSAMPLE 1  //8个Level，0代表140us，1代表204us，2代表332us，3代表588us，4代表1100us，5代表2116us，6代表4156us，7代表8244us

#define OPERATINGMODE 0 //芯片提供8个Level，库提供两种模式——连续电压电流采样（1）与连续电流（0）采样。

//float bsp_CurrentRead[5];									  //采样的结果，bsp_CurrentRead[0] 为1号电机电流，以此类推到4号电机，5为总采样ID
//float bsp_VoltageRead[5];									  //类似

int16_t bsp_CurrentRead[5];									  //采样的结果，bsp_CurrentRead[0] 为1号电机电流，以此类推到4号电机，5为总采样ID
int16_t bsp_VoltageRead[5];									  //类似


float CURRENT_LSB[5] = {0.25f, 0.255f, 0.248f, 0.2483f, 0.248f};	 //建议利用电子负载验证你的公式
float VOLTAGE_LSB[5] = {1.25f, 1.25f, 1.25f, 1.25f, 1.25f}; //建议利用电子负载验证你的公式
//uint8_t INA226_ID[5] = {0x9C, 0x82, 0x88, 0x8A, 0x80};		  //几个轮子的ID号码，某一位改为0将会跳过那一位
uint8_t INA226_ID[5] = {0x88, 0x82, 0x8A, 0x80, 0x9C};		  //几个轮子的ID号码，某一位改为0将会跳过那一位

static int16_t bsp_current_TimeOut = 0x00FF; //I2C通讯超时时间

int16_t JSCOPE_INA226_Current = 0;

int8_t bsp_current_CycleID = 1; //ID号，便于循环

static uint8_t I2CTx[8];
static uint8_t I2CRx[8];

static void Current_InitAssi(uint8_t ID);
static void Current_Read_Assistant(uint8_t ID);

void bsp_Current_Read(void) //为了方便非4个轮子的兵种，循环与执行分开。这个函数的做法是调用一次读取一个轮子电流。略加修改可以改成你想要的效果
{
	while (bsp_current_CycleID <= 4) //此处逻辑为读取四次，退出函数。可根据实际情况修改o(*￣▽￣*)o
	{
		memset(I2CRx,0,8);
		I2CTx[0] = 0x01; //Current
		HAL_I2C_Master_Transmit(&INA226I2C, INA226_ID[bsp_current_CycleID], &I2CTx[0], 1, bsp_current_TimeOut);
		//HAL_I2C_Master_Receive_IT(&INA226I2C, INA226_ID[bsp_current_CycleID],  &I2CRx[0], 2);
		HAL_I2C_Master_Receive(&INA226I2C, INA226_ID[bsp_current_CycleID], &I2CRx[0], 2, bsp_current_TimeOut);

		bsp_CurrentRead[bsp_current_CycleID] = (int16_t)(((I2CRx[0] << 8) + I2CRx[1])) * CURRENT_LSB[bsp_current_CycleID];
		
		// //debug
		// if (bsp_current_CycleID == 2)
		// 	JSCOPE_INA226_Current = (int16_t)(((I2CRx[0] << 8) + I2CRx[1])) * CURRENT_LSB[bsp_current_CycleID];
		// //debug
#if		OPERATINGMODE == 1
		I2CTx[0] = 0x02; //Voltage
		HAL_I2C_Master_Transmit(&INA226I2C, INA226_ID[bsp_current_CycleID], &I2CTx[0], 1, bsp_current_TimeOut);
		HAL_I2C_Master_Receive(&INA226I2C, INA226_ID[bsp_current_CycleID], &I2CRx[0], 2, bsp_current_TimeOut);
		bsp_VoltageRead[bsp_current_CycleID] = (int16_t)(((I2CRx[0] << 8) + I2CRx[1])) * VOLTAGE_LSB[bsp_current_CycleID];
#endif
		bsp_current_CycleID++;
	}
	bsp_current_CycleID = 0; //一个循环之后ID号归位
}

void bsp_Current_Init(void) //初始化，为了方便非4个轮子的兵种，循环与执行分开。
{

	for (uint8_t ID = 0; ID <= 4; ID++)
	{
		if (INA226_ID[ID] == 0)
			continue; //几个轮子的ID号码，某一位改为0将会跳过那一位

		memset(&I2CTx[0], 0, sizeof(I2CTx)); //发送Buffer清零
		uint16_t CONFIG = 0;
		CONFIG = CONFIG + (0x04 << 12);

		switch (NUMOFAVG)
		{
			case 1024:CONFIG = CONFIG + (0x07 << 9);break;
			case 512:CONFIG = CONFIG + (0x06 << 9);break;
			case 256:CONFIG = CONFIG + (0x05 << 9);break;
			case 128:CONFIG = CONFIG + (0x04 << 9);break;
			case 64:CONFIG = CONFIG + (0x03 << 9);break;
			case 16:CONFIG = CONFIG + (0x02 << 9);break;
			case 4:CONFIG = CONFIG + (0x01 << 9);break;
			case 1:CONFIG = CONFIG + (0x00 << 9);break;

		default:while (1);	//配置有问题
		break;
		}
		switch (TIMEOFSAMPLE)
		{
			case 7:CONFIG = CONFIG + (0x3F << 3);break;
			case 6:CONFIG = CONFIG + (0x36 << 3);break;
			case 5:CONFIG = CONFIG + (0x2D << 3);break;
			case 4:CONFIG = CONFIG + (0x24 << 3);break;
			case 3:CONFIG = CONFIG + (0x1B << 3);break;
			case 2:CONFIG = CONFIG + (0x12 << 3);break;
			case 1:CONFIG = CONFIG + (0x09 << 3);break;
			case 0:CONFIG = CONFIG + (0x00 << 3);break;

		default:while (1);	//配置有问题
		break;
		}


#if OPERATINGMODE == 1
CONFIG = CONFIG + 0x07;
#elif OPERATINGMODE == 0
CONFIG = CONFIG + 0x05;
#else
#error 请指定合法采样模式
#endif
		I2CTx[0] = 0x00;
		I2CTx[1] = CONFIG >> 8;
		I2CTx[2] = CONFIG & 0xFF;

		HAL_I2C_Master_Transmit(&INA226I2C, ID, &I2CTx[0], 3, bsp_current_TimeOut);

		memset(&I2CTx[0], 0, sizeof(I2CTx));
	}
}
