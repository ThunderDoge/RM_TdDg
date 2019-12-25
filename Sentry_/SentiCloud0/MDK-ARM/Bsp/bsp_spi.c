#include "bsp_spi.h"
#include "spi.h"
//��Ҫ�Լ�ת�ӵ�3���궨��
#define MPU_SPI hspi1
#define ICM_Disable()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)
#define ICM_Enable()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)



#ifdef USE_MAG
#define AK_Disable()   HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_SET)
#define AK_Enable()    HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_RESET)

uint8_t bsp_spi_MagAsa[3];//��ȡ������ASAУ���Ĵ���
#endif

/** 
* @brief   ���Ĵ�����д����
* @remarks 
* @retval  Null     
*/
static void bsp_spi_writereg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //��λ0λд
	ICM_Enable();   
	HAL_SPI_Transmit(&MPU_SPI, &regAddr,1,100);
	HAL_SPI_Transmit(&MPU_SPI, &data,1,100);
	ICM_Disable();
}	
/** 
* @brief   ���εĶ�ȡ
* @remarks һ��ֻ�ܶ�ȡһ���Ĵ�����ֵ����8λ
*/
uint8_t bsp_spi_ReadReg(uint8_t regAddr)
{
	uint8_t temp;
	bsp_spi_ReadRegs(regAddr,&temp,1);
	return temp;	
}

	/** 
* @brief   �ӼĴ����ж�����
* @remarks �����治һ����RW��ҪΪ1
*          ͬʱ����Ҫ�ȷ������ݣ�Ϲ���ѷ�8λ�����ˣ�
* @retval  ��ȡ����8λ����    
* @bug     ������Ҫ���ձ�׼��0xC0����������ȡ������Ҫ0x80
*/
void bsp_spi_ReadRegs(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  regAddr |= 0x80;
  ICM_Enable();
	HAL_SPI_Transmit(&MPU_SPI, &regAddr, 1, 100);
	HAL_SPI_Receive(&MPU_SPI, pBuff, length, 100);
	ICM_Disable();	
}
#ifdef USE_MAG

/** 
    * @brief ������д
*/
static uint8_t bsp_spi_magwrite(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //��λ0λд
	AK_Enable();   
	HAL_SPI_Transmit(&MPU_SPI, &regAddr,1,2000);
	if(HAL_SPI_Transmit(&MPU_SPI, &data,1,2000)!=HAL_OK)
		return 0;
	AK_Disable();
	return 1;
}	
/** 
    * @brief �����ƶ�ȡ
*/
uint8_t bsp_spi_MagReads(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  regAddr |= 0x80;
  AK_Enable();
	HAL_SPI_Transmit(&MPU_SPI, &regAddr, 1, 2000);
	if(HAL_SPI_Receive(&MPU_SPI, pBuff, length, 2000)!=HAL_OK)
		return 0;
	AK_Disable();	
	return 1;
}

/** 
    * @brief �����������
*/
void bsp_spi_MagTrig(void)
{
	bsp_spi_magwrite(AK8975_CNTL_REG,0x01);
}
/** 
* @brief  �����Ƴ�ʼ��
* @param[in]   �� 
* @retval  HAL_OK  �ɹ�  HAL_ERROR ʧ��
* @par ��־ 
*
*/
HAL_StatusTypeDef bsp_spi_Ak8975Init(void)
{
	static uint8_t ID;
	bsp_spi_magwrite(AK8975_I2CDIS_REG,0x1B);
	bsp_spi_MagTrig();
	bsp_spi_MagReads(AK8975_ASAX_REG,bsp_spi_MagAsa,3);
	bsp_spi_MagTrig();
	bsp_spi_MagReads(AK8975_WIA_REG,&ID,1);
	if(ID == 0x48)
	{
		bsp_spi_MagTrig();
		return HAL_OK;
	}
	else 
		return HAL_ERROR;
}
#endif
/** 
* @brief  ICM20602��ʼ��
* @param[in]   �� 
* @retval  HAL_OK  �ɹ�  HAL_ERROR ʧ��
* @par ��־ 
*
*/
HAL_StatusTypeDef bsp_spi_Icm20602Init(void)
{
	static uint8_t ID;
	bsp_spi_writereg(MPU_RA_PWR_MGMT_1,0x80);						//����ڲ��Ĵ���
	HAL_Delay(10);
	bsp_spi_writereg(MPU_RA_PWR_MGMT_1,0x01);           //����ʱ��Դ
	HAL_Delay(10);

	ID = bsp_spi_ReadReg(MPUREG_WHOAMI);                //WHO AM I!!!
	if(ID == MPU_WHOAMI_20602)
	{

		bsp_spi_writereg(MPU_RA_SIGNAL_PATH_RESET,0x03);  
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_USER_CTRL,0x01);					//��λ���Ĵ���
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_PWR_MGMT_2,0x00);					//���������Ǻͼ��ٶȼ�
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_SMPLRT_DIV,0);						//����Ƶ
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_CONFIG,ICM20602_LPF_20HZ);//��ͨ�˲�����
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_GYRO_CONFIG,(3 << 3));		//����������+-2000dps
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_ACCEL_CONFIG_1,0x11); 		//���ٶȼ�����+-2g
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_ACCEL_CONFIG_2,0x04);			//���ٶȼ��˲�����
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_LP_CONFIG,0x00);					//�رյ͹���
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_FIFO_EN,0x00);						//�ر�FIFO
		
		return HAL_OK;
	}	
	else
		return HAL_ERROR;
}


