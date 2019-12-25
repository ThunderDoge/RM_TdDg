#include "bsp_spi.h"
#include "spi.h"
//需要自己转接的3个宏定义
#define MPU_SPI hspi1
#define ICM_Disable()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)
#define ICM_Enable()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)



#ifdef USE_MAG
#define AK_Disable()   HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_SET)
#define AK_Enable()    HAL_GPIO_WritePin(AK_CS_GPIO_Port, AK_CS_Pin, GPIO_PIN_RESET)

uint8_t bsp_spi_MagAsa[3];//读取磁力计ASA校正寄存器
#endif

/** 
* @brief   往寄存器中写数据
* @remarks 
* @retval  Null     
*/
static void bsp_spi_writereg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //首位0位写
	ICM_Enable();   
	HAL_SPI_Transmit(&MPU_SPI, &regAddr,1,100);
	HAL_SPI_Transmit(&MPU_SPI, &data,1,100);
	ICM_Disable();
}	
/** 
* @brief   单次的读取
* @remarks 一次只能读取一个寄存器的值，即8位
*/
uint8_t bsp_spi_ReadReg(uint8_t regAddr)
{
	uint8_t temp;
	bsp_spi_ReadRegs(regAddr,&temp,1);
	return temp;	
}

	/** 
* @brief   从寄存器中读数据
* @remarks 与上面不一样，RW需要为1
*          同时，需要先发送数据（瞎几把发8位就行了）
* @retval  读取到的8位数据    
* @bug     磁力计要按照标准的0xC0才能连续读取，其他要0x80
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
    * @brief 磁力计写
*/
static uint8_t bsp_spi_magwrite(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f;  //首位0位写
	AK_Enable();   
	HAL_SPI_Transmit(&MPU_SPI, &regAddr,1,2000);
	if(HAL_SPI_Transmit(&MPU_SPI, &data,1,2000)!=HAL_OK)
		return 0;
	AK_Disable();
	return 1;
}	
/** 
    * @brief 磁力计读取
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
    * @brief 磁力计命令触发
*/
void bsp_spi_MagTrig(void)
{
	bsp_spi_magwrite(AK8975_CNTL_REG,0x01);
}
/** 
* @brief  磁力计初始化
* @param[in]   无 
* @retval  HAL_OK  成功  HAL_ERROR 失败
* @par 日志 
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
* @brief  ICM20602初始化
* @param[in]   无 
* @retval  HAL_OK  成功  HAL_ERROR 失败
* @par 日志 
*
*/
HAL_StatusTypeDef bsp_spi_Icm20602Init(void)
{
	static uint8_t ID;
	bsp_spi_writereg(MPU_RA_PWR_MGMT_1,0x80);						//清除内部寄存器
	HAL_Delay(10);
	bsp_spi_writereg(MPU_RA_PWR_MGMT_1,0x01);           //配置时钟源
	HAL_Delay(10);

	ID = bsp_spi_ReadReg(MPUREG_WHOAMI);                //WHO AM I!!!
	if(ID == MPU_WHOAMI_20602)
	{

		bsp_spi_writereg(MPU_RA_SIGNAL_PATH_RESET,0x03);  
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_USER_CTRL,0x01);					//复位各寄存器
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_PWR_MGMT_2,0x00);					//开启陀螺仪和加速度计
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_SMPLRT_DIV,0);						//不分频
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_CONFIG,ICM20602_LPF_20HZ);//低通滤波配置
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_GYRO_CONFIG,(3 << 3));		//陀螺仪量程+-2000dps
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_ACCEL_CONFIG_1,0x11); 		//加速度计量程+-2g
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_ACCEL_CONFIG_2,0x04);			//加速度计滤波配置
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_LP_CONFIG,0x00);					//关闭低功耗
		HAL_Delay(10);
		bsp_spi_writereg(MPU_RA_FIFO_EN,0x00);						//关闭FIFO
		
		return HAL_OK;
	}	
	else
		return HAL_ERROR;
}


