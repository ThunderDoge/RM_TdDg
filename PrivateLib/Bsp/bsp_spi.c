/** 
* @brief    SPI板级支持包
* @details  SDI相关硬件函数，主要包含读写MPU9250内部寄存器，初始化MPU9250
* @author   郭俊辉
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*/

#include "bsp_spi.h"

/**
* @brief  MPU9250片选
* @details  通过SPI总线片选9250芯片
* @param  NULL
* @retval  NULL
*/
static void bsp_SPI_MPU9250_ChipSelect(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);	/*片选MPU9250*/
}

/**
* @brief  MPU9250放弃片选
* @details  通过SPI总线放弃片选9250芯片
* @param  NULL
* @retval  NULL
*/
static void bsp_SPI_MPU9250_NoChipSelect(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);	/*放弃片选MPU9250*/
}

/**
* @brief  MPU9250写寄存器
* @details  通过SPI总线向MPU9250寄存器写入数据
* @param  Regaddr 寄存器地址 Data写入的数据
* @retval  NULL
*/
static void bsp_SPI_MPU9250_Writereg(uint8_t Regaddr, uint8_t Data)
{
	bsp_SPI_MPU9250_ChipSelect();
	HAL_SPI_Transmit(&hspi1, &Regaddr,1,0x05);
	HAL_SPI_Transmit(&hspi1, &Data,1,0x05);
	bsp_SPI_MPU9250_NoChipSelect();
	HAL_Delay(5);
}

/**
* @brief  读取MPU9250寄存器值
* @details  通过SPI总线读取MPU9250中寄存器的值
* @param  uint8_t Regadrr 寄存器地址
* @retval  uint8_t Spi_Rx 读取得到的数据
*/
uint8_t bsp_SPI_MPU9250_Readreg(uint8_t Regadrr)
{
	static uint8_t Spi_Tx,Spi_Rx;
  Spi_Tx = Regadrr | 0x80;
  bsp_SPI_MPU9250_ChipSelect();
	HAL_SPI_TransmitReceive(&hspi1, &Spi_Tx, &Spi_Rx, 1, 0x05);
	HAL_SPI_Receive(&hspi1, &Spi_Rx, 1, 0x05);
	bsp_SPI_MPU9250_NoChipSelect();
	return Spi_Rx;
}

/**
* @brief  读取MPU9250多个寄存器值
* @details  通过SPI总线读取MPU9250中连续多个寄存器得值
* @param  Regadrr 寄存器地址 Read_Buffer 传出数据的数组 Length寄存器长度
* @retval  NULL
*/
void bsp_SPI_MPU9250_Readregs(uint8_t Regadrr, uint8_t* Read_Buffer, uint8_t Length)
{
	static uint8_t Spi_Tx,Spi_Rx;
	Spi_Tx = Regadrr | 0x80;
  bsp_SPI_MPU9250_ChipSelect();
	HAL_SPI_TransmitReceive(&hspi1, &Spi_Tx, &Spi_Rx, 1, 0x05);
	HAL_SPI_Receive(&hspi1, Read_Buffer, Length, 0x05);
	bsp_SPI_MPU9250_NoChipSelect();
}

/**
* @brief  磁力计寄存器写入
* @details  通过SPI总线对MPU9250内部磁力计寄存器写入数据
* @param  
* @retval  
*/
void bsp_SPI_AK8963_Writereg(uint8_t Regaddr, uint8_t data)
{
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x0C); //写
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_REG,Regaddr);
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x81);	
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_DO,data);
}

/**
* @brief  磁力计寄存器读取
* @details  通过SPI总线读取MPU9250内部的磁力计寄存器数据
* @param  Readaddr 寄存器地址
* @retval  读取到的数据
*/
uint8_t bsp_SPI_AK8963_Readreg(uint8_t Regaddr)
{
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C); //读
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_REG, Regaddr);  // 地址
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x81);  // 读到MPU6500_EXT_SENS_DATA_00	
	return  bsp_SPI_MPU9250_Readreg(MPU6500_EXT_SENS_DATA_00);
}

/**
* @brief  磁力计寄存器连续读取
* @details  通过SPI读取MPU9250内部连续多个寄存器数据
* @param  Regadrr 寄存器地址 Read_Buff传出数据的数组 Length读取长度
* @retval  NULL
*/
void bsp_SPI_AK8963_Readregs(uint8_t Regaddr,uint8_t *Read_Buff,uint8_t Length)
{
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C); //读
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_REG, Regaddr);  // 地址
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x80+Length);  // 读到MPU6500_EXT_SENS_DATA_00
	bsp_SPI_MPU9250_Readregs(MPU6500_EXT_SENS_DATA_00,Read_Buff,Length);
}

uint8_t AK8963_ASA[3];
/**
* @brief  MPU9250初始化
* @details  通过SPI总线对MPU9250内部寄存器写入数据做初始化，当ID号检测不通过时，将不会进行初始化而是进入死循环
* @param  NULL
* @retval  NULL
*/
void bsp_MPU9250_Init(void)
{
	while(bsp_SPI_MPU9250_Readreg(0X75) == 0X71);			/*MPU9250的ID检测*/
	
	bsp_SPI_MPU9250_Writereg(MPU6500_PWR_MGMT_1,0x80);   // 清除内部寄存器为默认设置 
	bsp_SPI_MPU9250_Writereg(MPU6500_PWR_MGMT_1,0x01);   // Clock Source
	bsp_SPI_MPU9250_Writereg(MPU6500_CONFIG,0x04);       // 低通滤波频率5  250HZ (00) 184 92 41 20 10 5 3600 
	bsp_SPI_MPU9250_Writereg(MPU6500_USER_CTRL,0x20);    // SPI + I2C master Enable AUX
/*************************** 初始化陀螺仪和加速度计 **************************/	
	bsp_SPI_MPU9250_Writereg(MPU6500_SMPLRT_DIV,0x00);      // sample rate,this is the updata rate of sensor register 1000Hz
	bsp_SPI_MPU9250_Writereg(MPU6500_GYRO_CONFIG,0x18);     // +-2000dps 不自检
	bsp_SPI_MPU9250_Writereg(MPU6500_ACCEL_CONFIG,0x00);    // +-2G
	bsp_SPI_MPU9250_Writereg(MPU6500_ACCEL_CONFIG_2,0x06);  // 加速计低通滤波器设置 ，5Hz
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_MST_CTRL,0x5D);    // I2C Speed 400 kHz there is a stop between reads.	
/*************************** 初始化磁力计 **************************/		
	bsp_SPI_AK8963_Writereg(AK8963_CNTL2,0x01);         // reset
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x10);         // Power-down mode
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x1F);   			 // Fuse ROM access mode
	bsp_SPI_AK8963_Readregs(AK8963_ASAX,AK8963_ASA,3);  // AK8963 get calibration data
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x10);         // Power-down mode
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C);       // 读
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_REG, AK8963_ST1);  // 地址
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x88);	
  bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x16);         // Continuous measurement mode 2
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_SLV4_CTRL,0x09);  //When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	bsp_SPI_MPU9250_Writereg(MPU6500_I2C_MST_DELAY_CTRL,0x81); //[7]Delays shadowing of external sensor data until all data is received 	
}
