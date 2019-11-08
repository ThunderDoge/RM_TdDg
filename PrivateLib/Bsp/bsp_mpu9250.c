/**
*   MPU6500����16λ���ٶ�AD���������16λ������AD���������16λ������AD���
*   �����ǿ�ѡ����250,500,1000,2000(dps)�����ٶȼ�2,4,6,8,16(g)��������4800uT
*   ͨ�Ų���400KHz��I2C��1MHz��SPI����������ٶȣ�����SPI��20MHz��ģʽ��ֱ�Ӷ�ȡ���������жϼĴ���
*   9250Ƭѡ���ű����ΪSPI1_nCS
**/
#include "bsp_mpu9250.h"

static uint8_t tx, rx;
/** 
* @brief   ���Ĵ�����д����
* @remarks ��һ��8λ����RW,MS,AD5`AD0���ڶ�����λ����D17`D10��
* @retval  Null     
*/
static void SPI_MPU_Write_Reg(uint8_t regAddr, uint8_t data){
	Slaver_Enable();   
	HAL_SPI_Transmit(&hspi1, &regAddr,1,0x05);
	HAL_SPI_Transmit(&hspi1, &data,1,0x05);
	Slaver_Disable();
	HAL_Delay(5);
}	
/** 
* @brief   ���εĶ�ȡ
* @remarks һ��ֻ�ܶ�ȡһ���Ĵ�����ֵ����8λ
*/
uint8_t SPI_MPU_Read(uint8_t regAddr){
	uint8_t temp;
	SPI_MPU_Read_Regs(regAddr,&temp,1);
	return temp;	
}

	/** 
* @brief   �ӼĴ����ж�����
* @remarks �����治һ����RW��ҪΪ1
*          ͬʱ����Ҫ�ȷ������ݣ�Ϲ���ѷ�8λ�����ˣ�
* @retval  ��ȡ����8λ����    
* @bug     ������Ҫ���ձ�׼��0xC0����������ȡ������Ҫ0x80
*/
void SPI_MPU_Read_Regs(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  tx = regAddr | 0x80;
		Slaver_Enable();
	HAL_SPI_TransmitReceive(&MPU_SPI, &tx, &rx, 1, 0x05);
	HAL_SPI_Receive(&MPU_SPI, pBuff, length, 0x05);
	Slaver_Disable();	
}
/** 
* @brief   ������д����
* @remarks ͨ��SPI��������д����
* @pra    
*/
void AK8963_Write_Reg(uint8_t addr, uint8_t data)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x0C); //д
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG,addr);
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x81);	
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_DO,data);
}
/** 
* @brief   �����ƶ�����
* @remarks ͨ��SPI��ȡ������һ���Ĵ�������
* @pra    
*/
uint8_t AK8963_Read_Reg(uint8_t addr)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //��
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, addr);  // ��ַ
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x81);  // ����MPU6500_EXT_SENS_DATA_00	
	return  SPI_MPU_Read(MPU6500_EXT_SENS_DATA_00);
}
/** 
* @brief   �����ƶ�����
* @remarks ͨ��SPI��ȡ�����ƶ���Ĵ�������
* @pra    
*/
void AK8963_Read_Regs(uint8_t addr,uint8_t *pBuff,uint8_t len)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //��
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, addr);  // ��ַ
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x80+len);  // ����MPU6500_EXT_SENS_DATA_00
	SPI_MPU_Read_Regs(MPU6500_EXT_SENS_DATA_00,pBuff,len);
}

void Mag_Read(uint8_t *pBuff)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //��
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, AK8963_HXL);  // ��ַ
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x86);  // ����MPU6500_EXT_SENS_DATA_00
	SPI_MPU_Read_Regs(MPU6500_EXT_SENS_DATA_00,pBuff,1);
	AK8963_Write_Reg(AK8963_CNTL1,0x16);
}

/** 
* @brief   ��MPU6500��������
* @remarks ��Ҫ�ڹ����г�ʼ��
*/
//uint8_t AK8963_ASA[3];
void MPU9250_Init(void)
{
	SPI_MPU_Write_Reg(MPU6500_PWR_MGMT_1,0x80);   // ����ڲ��Ĵ���ΪĬ������ 
	SPI_MPU_Write_Reg(MPU6500_PWR_MGMT_1,0x01);   // Clock Source
	SPI_MPU_Write_Reg(MPU6500_CONFIG,0x04);       // ��ͨ�˲�Ƶ��5  250HZ (00) 184 92 41 20 10 5 3600 
	SPI_MPU_Write_Reg(MPU6500_USER_CTRL,0x20);    // SPI + I2C master Enable AUX
/*************************** ��ʼ�������Ǻͼ��ٶȼ� **************************/	
	SPI_MPU_Write_Reg(MPU6500_SMPLRT_DIV,0x00);      // sample rate,this is the updata rate of sensor register 1000Hz
	SPI_MPU_Write_Reg(MPU6500_GYRO_CONFIG,0x18);     // +-2000dps ���Լ�
	SPI_MPU_Write_Reg(MPU6500_ACCEL_CONFIG,0x10);    // +-8G
	SPI_MPU_Write_Reg(MPU6500_ACCEL_CONFIG_2,0x0B);  // ���ټƵ�ͨ�˲������� 99Hz 
	SPI_MPU_Write_Reg(MPU6500_I2C_MST_CTRL,0x5D);    // I2C Speed 400 kHz there is a stop between reads.	
/*************************** ��ʼ�������� **************************/		
	AK8963_Write_Reg(AK8963_CNTL2,0x01);         // reset
	AK8963_Write_Reg(AK8963_CNTL1,0x10);         // Power-down mode
	AK8963_Write_Reg(AK8963_CNTL1,0x1F);   			 // Fuse ROM access mode
	AK8963_Read_Regs(AK8963_ASAX,AK8963_ASA,3);  // AK8963 get calibration data
	AK8963_Write_Reg(AK8963_CNTL1,0x10);         // Power-down mode
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C);       // ��
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, AK8963_ST1);  // ��ַ
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x88);	
  AK8963_Write_Reg(AK8963_CNTL1,0x16);         // Continuous measurement mode 2
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV4_CTRL,0x09);  //When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	SPI_MPU_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL,0x81); //[7]Delays shadowing of external sensor data until all data is received 	
}
