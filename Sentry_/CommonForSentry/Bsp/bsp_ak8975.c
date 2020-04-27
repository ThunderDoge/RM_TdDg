/**
* @file bsp_ak8975.c
* @brief 
* @author Asn (921576434@qq.com)
* @date 2020.04.10
* @version 1.0
* @copyright Copyright (c) 2020
* @par 日志:
*		v1.0 从原bsp_spi分离而来
*/
#include "bsp_ak8975.h"
#include "spi.h"



#ifdef USE_MAG
#define AK8975_SPI hspi1
#define AK_Disable()   HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_SET)
#define AK_Enable()    HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_RESET)

uint8_t bsp_ak8975_Asa[3];//读取磁力计ASA校正寄存器


/** 
* @brief 磁力计写函数
*/
static uint8_t bsp_ak8975_WriteReg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //首位0位写
	AK_Enable();   
	HAL_SPI_Transmit(&AK8975_SPI, &regAddr,1,2000);
	if(HAL_SPI_Transmit(&AK8975_SPI, &data,1,2000)!=HAL_OK)
		return 0;
	AK_Disable();
	return 1;
}	
/** 
* @brief 磁力计读取函数
*/
uint8_t bsp_ak8975_ReadRegs(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  regAddr |= 0x80;
  AK_Enable();
	HAL_SPI_Transmit(&AK8975_SPI, &regAddr, 1, 2000);
	if(HAL_SPI_Receive(&AK8975_SPI, pBuff, length, 2000)!=HAL_OK)
		return 0;
	AK_Disable();	
	return 1;
}

/** 
* @brief 磁力计命令触发
*/
void bsp_ak8975_Trig(void)
{
	bsp_ak8975_WriteReg(AK8975_CNTL_REG,0x01);
}
/** 
* @brief 磁力计初始化
* @retval  HAL_OK  成功  HAL_ERROR 失败
*/
/**
* @brief 磁力计初始化
* @retval HAL_OK  成功
* @retval 其他 异常退出
*/
HAL_StatusTypeDef bsp_ak8975_Init(void)
{
	static uint8_t ID;
	bsp_ak8975_WriteReg(AK8975_I2CDIS_REG,0x1B);
	bsp_ak8975_Trig();
	bsp_ak8975_ReadRegs(AK8975_ASAX_REG,bsp_ak8975_Asa,3);
	bsp_ak8975_Trig();
	bsp_ak8975_ReadRegs(AK8975_WIA_REG,&ID,1);
	if(ID == 0x48)
	{
		bsp_ak8975_Trig();
		return HAL_OK;
	}
	else 
		return HAL_ERROR;
}
#endif
