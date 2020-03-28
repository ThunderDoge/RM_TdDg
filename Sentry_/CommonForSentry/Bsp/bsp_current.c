/**
	****************************(C) COPYRIGHT KisonHe****************************
	* @file       bsp_current.cpp
	* @brief      KisonHe �ĵ�������
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

#define CURRENT_LSB (0.25f) //�������費ͬ�ᵼ�¼��㹫ʽ��ͬ��ѯ����ĵ�·���������������ʽ��
														//�������õ��Ӹ�����֤��Ĺ�ʽ
#define INA226I2C (hi2c3)

uint8_t INA226_ID[5] = {0, 0x80, 0x88, 0x8A, 0x80}; //�������ӵ�ID���룬

int16_t bsp_CurrentRead[5]; //�����Ľ����bsp_CurrentRead[1] Ϊ1�ŵ���������Դ����Ƶ�4�ŵ��
int16_t bsp_VoltageRead[5]; //����

static int32_t bsp_current_TempCurrent = 0;
static int32_t bsp_current_TempVoltage = 0;
//int16_t bsp_current_ConfigRead = 0;			//��226��ȡ�������ã�����DEBUGʹ�á���ȡ��������Init�����У��ѱ�ע��
static int16_t bsp_current_TimeOut = 0x00FF; //I2CͨѶ��ʱʱ��

//int16_t bsp_current_TempID = INA226_No4; //ID�ţ�����ѭ��
int8_t bsp_current_CycleID = 1;			 //ID�ţ�����ѭ��

static uint8_t I2CTx[8];
static uint8_t I2CRx[8];


static void Current_InitAssi(uint8_t ID);
static void Current_Read_Assistant(uint8_t ID);


void bsp_Current_Read(void) //Ϊ�˷����4�����ӵı��֣�ѭ����ִ�зֿ�����������������ǵ���һ�ζ�ȡһ�����ӵ������Լ��޸Ŀ��Ըĳ�����Ҫ��Ч��
{
	
		Current_Read_Assistant(bsp_current_CycleID); //ʵ�ʶ�ȡ����`
	
	bsp_current_CycleID = 1; //һ��ѭ��֮��ID�Ź�λ
}

void bsp_Current_Init(void) //��ʼ����Ϊ�˷����4�����ӵı��֣�ѭ����ִ�зֿ���
{
	Current_InitAssi(INA226_ID[1]);
	
}


static void Current_InitAssi(uint8_t ID)
{
	memset(&I2CTx[0], 0, sizeof(I2CTx)); //����Buffer����

	//1024�ξ�ֵ��140usһ�β����������������ѹģʽ
	I2CTx[0] = 0x00;
//	I2CTx[1] = 0x4E;
//	To increase sampling rate
	I2CTx[1] = 0x46;
	I2CTx[2] = 0x07;

	HAL_I2C_Master_Transmit(&INA226I2C, ID, &I2CTx[0], 3, bsp_current_TimeOut);

	memset(&I2CTx[0], 0, sizeof(I2CTx));

	//��226��ȡ�������ã�����DEBUG���⡣
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

	bsp_CurrentRead[ID] = bsp_current_TempCurrent; //����ʱ�������ȡ������
	bsp_VoltageRead[ID] = bsp_current_TempVoltage; //����ʱ�������ȡ������
}
