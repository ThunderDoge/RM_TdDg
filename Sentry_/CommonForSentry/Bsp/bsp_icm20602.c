/**
* @file bsp_icm20602.c
* @brief 陀螺仪配置
* @author Asn (921576434@qq.com)
* @date 2019.10
* @version 1.0
* @copyright Copyright (c) RM2020电控
* @par 日志:
*         v1.0 创建此文件
*/
#include "bsp_icm20602.h"
#include "spi.h"
//需要自己转接的3个宏定义
#define ICM20602_SPI hspi1
#define ICM_Disable()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)
#define ICM_Enable()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)


/** 
* @brief 陀螺仪写函数   
*/
static void bsp_icm20602_WriteReg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //首位0位写
	ICM_Enable();   
	HAL_SPI_Transmit(&ICM20602_SPI, &regAddr,1,100);
	HAL_SPI_Transmit(&ICM20602_SPI, &data,1,100);
	ICM_Disable();
}	
/** 
* @brief 陀螺仪单次读取函数
*/
uint8_t bsp_icm20602_ReadReg(uint8_t regAddr)
{
	uint8_t temp;
	bsp_icm20602_ReadRegs(regAddr,&temp,1);
	return temp;	
}
/**
* @brief 陀螺仪读取函数
*/
void bsp_icm20602_ReadRegs(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  regAddr |= 0x80;
  ICM_Enable();
	HAL_SPI_Transmit(&ICM20602_SPI, &regAddr, 1, 100);
	HAL_SPI_Receive(&ICM20602_SPI, pBuff, length, 100);
	ICM_Disable();	
}



/**
* @brief ICM20602初始化
* @retval HAL_OK  成功
* @retval 其他 异常退出
* @return HAL_StatusTypeDef 
*/
HAL_StatusTypeDef bsp_icm20602_Init(void)
{
	static uint8_t ID;
	bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_1,0x80);						//清除内部寄存器
	HAL_Delay(10);
	bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_1,0x01);           //配置时钟源
	HAL_Delay(10);

	ID = bsp_icm20602_ReadReg(MPUREG_WHOAMI);                //WHO AM I!!!
	if(ID == MPU_WHOAMI_20602)
	{

		bsp_icm20602_WriteReg(MPU_RA_SIGNAL_PATH_RESET,0x03);  
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_USER_CTRL,0x01);					//复位各寄存器
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_2,0x00);					//开启陀螺仪和加速度计
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_SMPLRT_DIV,0);						//不分频
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_CONFIG,ICM20602_LPF_20HZ);//低通滤波配置
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_GYRO_CONFIG,(3 << 3));		//陀螺仪量程+-2000dps
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_ACCEL_CONFIG_1,(3 << 3)); 		//加速度计量程+-16g
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_ACCEL_CONFIG_2,0x04);			//加速度计滤波配置
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_LP_CONFIG,0x00);					//关闭低功耗
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_FIFO_EN,0x00);						//关闭FIFO
		
		return HAL_OK;
	}	
	else
		return HAL_ERROR;
}


