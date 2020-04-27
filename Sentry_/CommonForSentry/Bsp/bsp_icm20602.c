/**
* @file bsp_icm20602.c
* @brief ����������
* @author Asn (921576434@qq.com)
* @date 2019.10
* @version 1.0
* @copyright Copyright (c) RM2020���
* @par ��־:
*         v1.0 �������ļ�
*/
#include "bsp_icm20602.h"
#include "spi.h"
//��Ҫ�Լ�ת�ӵ�3���궨��
#define ICM20602_SPI hspi1
#define ICM_Disable()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)
#define ICM_Enable()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)


/** 
* @brief ������д����   
*/
static void bsp_icm20602_WriteReg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //��λ0λд
	ICM_Enable();   
	HAL_SPI_Transmit(&ICM20602_SPI, &regAddr,1,100);
	HAL_SPI_Transmit(&ICM20602_SPI, &data,1,100);
	ICM_Disable();
}	
/** 
* @brief �����ǵ��ζ�ȡ����
*/
uint8_t bsp_icm20602_ReadReg(uint8_t regAddr)
{
	uint8_t temp;
	bsp_icm20602_ReadRegs(regAddr,&temp,1);
	return temp;	
}
/**
* @brief �����Ƕ�ȡ����
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
* @brief ICM20602��ʼ��
* @retval HAL_OK  �ɹ�
* @retval ���� �쳣�˳�
* @return HAL_StatusTypeDef 
*/
HAL_StatusTypeDef bsp_icm20602_Init(void)
{
	static uint8_t ID;
	bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_1,0x80);						//����ڲ��Ĵ���
	HAL_Delay(10);
	bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_1,0x01);           //����ʱ��Դ
	HAL_Delay(10);

	ID = bsp_icm20602_ReadReg(MPUREG_WHOAMI);                //WHO AM I!!!
	if(ID == MPU_WHOAMI_20602)
	{

		bsp_icm20602_WriteReg(MPU_RA_SIGNAL_PATH_RESET,0x03);  
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_USER_CTRL,0x01);					//��λ���Ĵ���
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_PWR_MGMT_2,0x00);					//���������Ǻͼ��ٶȼ�
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_SMPLRT_DIV,0);						//����Ƶ
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_CONFIG,ICM20602_LPF_20HZ);//��ͨ�˲�����
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_GYRO_CONFIG,(3 << 3));		//����������+-2000dps
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_ACCEL_CONFIG_1,(3 << 3)); 		//���ٶȼ�����+-16g
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_ACCEL_CONFIG_2,0x04);			//���ٶȼ��˲�����
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_LP_CONFIG,0x00);					//�رյ͹���
		HAL_Delay(10);
		bsp_icm20602_WriteReg(MPU_RA_FIFO_EN,0x00);						//�ر�FIFO
		
		return HAL_OK;
	}	
	else
		return HAL_ERROR;
}


