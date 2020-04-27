/**
* @file bsp_ak8975.c
* @brief 
* @author Asn (921576434@qq.com)
* @date 2020.04.10
* @version 1.0
* @copyright Copyright (c) 2020
* @par ��־:
*		v1.0 ��ԭbsp_spi�������
*/
#include "bsp_ak8975.h"
#include "spi.h"



#ifdef USE_MAG
#define AK8975_SPI hspi1
#define AK_Disable()   HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_SET)
#define AK_Enable()    HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_RESET)

uint8_t bsp_ak8975_Asa[3];//��ȡ������ASAУ���Ĵ���


/** 
* @brief ������д����
*/
static uint8_t bsp_ak8975_WriteReg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //��λ0λд
	AK_Enable();   
	HAL_SPI_Transmit(&AK8975_SPI, &regAddr,1,2000);
	if(HAL_SPI_Transmit(&AK8975_SPI, &data,1,2000)!=HAL_OK)
		return 0;
	AK_Disable();
	return 1;
}	
/** 
* @brief �����ƶ�ȡ����
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
* @brief �����������
*/
void bsp_ak8975_Trig(void)
{
	bsp_ak8975_WriteReg(AK8975_CNTL_REG,0x01);
}
/** 
* @brief �����Ƴ�ʼ��
* @retval  HAL_OK  �ɹ�  HAL_ERROR ʧ��
*/
/**
* @brief �����Ƴ�ʼ��
* @retval HAL_OK  �ɹ�
* @retval ���� �쳣�˳�
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
