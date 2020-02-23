/**
	****************************(C) COPYRIGHT KisonHe****************************
	* @file       bsp_current.cpp
	* @brief      KisonHe 的电流采样
	* @note
	* @history
	*  Version    Date            Author          Modification
	*  V1.0.0     2019-11-26      KisonHe			    1. 
	*
	**************************(C) COPYRIGHT KisonHe****************************
	*/

#include "bsp_current.h"
#include "main.h"
#include "i2c.h"

#include <stdlib.h>
#include <string.h>

#define CURRENT_LSB (0.25f) //采样电阻不同会导致计算公式不同，询问你的电路队友来计算采样公式。
														//建议利用电子负载验证你的公式
#define INA226I2C (hi2c3)

uint8_t INA226_ID[5] = {0, 0x80, 0x88, 0x8A, 0x80}; //几个轮子的ID号码，

int16_t bsp_CurrentRead[5]; //采样的结果，bsp_CurrentRead[1] 为1号电机电流，以此类推到4号电机
int16_t bsp_VoltageRead[5]; //类似

static int32_t bsp_current_TempCurrent = 0;
static int32_t bsp_current_TempVoltage = 0;
//int16_t bsp_current_ConfigRead = 0;			//从226读取到的配置，方便DEBUG使用。读取的那行在Init函数中，已被注释
static int16_t bsp_current_TimeOut = 0x00FF; //I2C通讯超时时间

//int16_t bsp_current_TempID = INA226_No4; //ID号，便于循环
int8_t bsp_current_CycleID = 1;			 //ID号，便于循环

static uint8_t I2CTx[8];
static uint8_t I2CRx[8];


static void Current_InitAssi(uint8_t ID);
static void Current_Read_Assistant(uint8_t ID);


void bsp_Current_Read(void) //为了方便非4个轮子的兵种，循环与执行分开。这个函数的做法是调用一次读取一个轮子电流。略加修改可以改成你想要的效果
{
	
		Current_Read_Assistant(bsp_current_CycleID); //实际读取数据`
	
	bsp_current_CycleID = 1; //一个循环之后ID号归位
}

void bsp_Current_Init(void) //初始化，为了方便非4个轮子的兵种，循环与执行分开。
{
	Current_InitAssi(INA226_ID[1]);
	
}


static void Current_InitAssi(uint8_t ID)
{
	memset(&I2CTx[0], 0, sizeof(I2CTx)); //发送Buffer清零

	//1024次均值，140us一次采样，持续电流与电压模式
	I2CTx[0] = 0x00;
//	I2CTx[1] = 0x4E;
//	To increase sampling rate
	I2CTx[1] = 0x46;
	I2CTx[2] = 0x07;

	HAL_I2C_Master_Transmit(&INA226I2C, ID, &I2CTx[0], 3, bsp_current_TimeOut);

	memset(&I2CTx[0], 0, sizeof(I2CTx));

	//从226读取到的配置，方便DEBUG问题。
	// I2CTx[0] = 0x00; //Confige Register
	// HAL_I2C_Master_Transmit(&INA226I2C, bsp_current_TempID, &I2CTx[0], 1, bsp_current_TimeOut);
	// HAL_I2C_Master_Receive(&INA226I2C, bsp_current_TempID, &I2CRx[0], 2, bsp_current_TimeOut);
	// bsp_current_ConfigRead = ((I2CRx[0] << 8) + I2CRx[1]);
}


static void Current_Read_Assistant(uint8_t ID)
{

	I2CTx[0] = 0x01; //Current
	HAL_I2C_Master_Transmit(&INA226I2C, INA226_ID[ID], &I2CTx[0], 1, bsp_current_TimeOut);
	HAL_I2C_Master_Receive(&INA226I2C, INA226_ID[ID], &I2CRx[0], 2, bsp_current_TimeOut);

	bsp_current_TempCurrent = (int16_t)(((I2CRx[0] << 8) + I2CRx[1])) * CURRENT_LSB;

	I2CTx[0] = 0x02; //Voltage
	HAL_I2C_Master_Transmit(&INA226I2C, INA226_ID[ID], &I2CTx[0], 1, bsp_current_TimeOut);
	HAL_I2C_Master_Receive(&INA226I2C, INA226_ID[ID], &I2CRx[0], 2, bsp_current_TimeOut);
	bsp_current_TempVoltage = (int16_t)(((I2CRx[0] << 8) + I2CRx[1])) * 1.25;

	bsp_CurrentRead[ID] = bsp_current_TempCurrent; //把临时量存入读取数组中
	bsp_VoltageRead[ID] = bsp_current_TempVoltage; //把临时量存入读取数组中
}
