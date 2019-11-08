#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "spi.h"

uint8_t bsp_SPI_MPU9250_Readreg(uint8_t Regadrr);
void bsp_SPI_MPU9250_Readregs(uint8_t Regadrr, uint8_t* Read_Buffer, uint8_t Length);
void bsp_SPI_AK8963_Writereg(uint8_t Regaddr, uint8_t data);
uint8_t bsp_SPI_AK8963_Readreg(uint8_t Regaddr);
void bsp_SPI_AK8963_Readregs(uint8_t Regaddr,uint8_t *Read_Buff,uint8_t Length);
void bsp_MPU9250_Init(void);

/*MPU9250�ڲ��Ĵ�������*/
#define MPU6500_I2C_READ            0x80
#define MPU6500_I2C_SLV4_EN         0x80
#define MPU6500_I2C_SLV4_DONE       0X40
#define MPU6500_I2C_SLV4_NACK       0X10
#define MPU6500_I2C_IF_DIS 					(0x10)
#define MPU6500_I2C_MST_EN				  (0x20)
#define MPU6500_CLOCK_PLLGYROZ 			0x03
#define MPU6500_XYZ_GYRO 						0xC7
#define MPU6500_XYZ_ACCEL 					0xF8
//#define MPU6500_I2C_ADDR            0x68  //��ȡʱ������һλ ��0xD0
#define SELF_TEST_X_GYRO            0x00
#define MPU6500_SELF_TEST_XG        0x00
#define MPU6500_SELF_TEST_YG        0x01
#define MPU6500_SELF_TEST_ZG        0x02
#define MPU6500_SELF_TEST_XA        0x0D
#define MPU6500_SELF_TEST_YA        0x0E
#define MPU6500_SELF_TEST_ZA        0x0F
#define MPU6500_XG_OFFSET_H         0x13
#define MPU6500_XG_OFFSET_L         0x14
#define MPU6500_YG_OFFSET_H         0x15
#define MPU6500_YG_OFFSET_L         0x16
#define MPU6500_ZG_OFFSET_H         0x17
#define MPU6500_ZG_OFFSET_L         0x18
#define MPU6500_SMPLRT_DIV          0x19  //�����ǲ���Ƶ�ʣ�����ֵ��0x07(125Hz) =1000/(1 + SMPLRT_DIV)
#define MPU6500_CONFIG              0x1A  //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz) 00(250),01(184),02(92),03(41),04(20),ֻ��0��7��ʱ�����Ƶ��Ϊ8KHz
#define MPU6500_GYRO_CONFIG         0x1B  //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s) 10(1000),08(500)
#define MPU6500_ACCEL_CONFIG        0x1C  //���ٶȼ��Լ켰��Χ������ֵ��0x18(���Լ죬16g)  00(2) 08(4) 10(8)
#define MPU6500_ACCEL_CONFIG_2      0x1D  //���ٶȼƵ�ͨ�˲���, ����ֵ��0x06(��ֹƵ��5Hz)  00/1(218.1) 02(99) 3(44.8)
#define MPU6500_WOM_THR             0x1F
#define MPU6500_FIFO_EN             0x23
#define MPU6500_I2C_MST_CTRL        0x24
#define MPU6500_I2C_SLV0_ADDR       0x25  //��λΪ1ʱ��ȡ��Ϊ0ʱд��6:0ΪPhysical address of I2C slave 0
#define MPU6500_I2C_SLV0_REG        0x26  //I2C slave 0 register address from where to begin data transfer
#define MPU6500_I2C_SLV0_CTRL       0x27  //��[7]Ϊ1������ȡ�����ݴ���EXT_SENS_DATA_00��[3:0]Number of bytes to be read from I2C slave 0 ,
#define MPU6500_I2C_SLV1_ADDR       0x28
#define MPU6500_I2C_SLV1_REG        0x29
#define MPU6500_I2C_SLV1_CTRL       0x2A
#define MPU6500_I2C_SLV2_ADDR       0x2B
#define MPU6500_I2C_SLV2_REG        0x2C
#define MPU6500_I2C_SLV2_CTRL       0x2D
#define MPU6500_I2C_SLV3_ADDR       0x2E
#define MPU6500_I2C_SLV3_REG        0x2F
#define MPU6500_I2C_SLV3_CTRL       0x30
#define MPU6500_I2C_SLV4_ADDR       0x31
#define MPU6500_I2C_SLV4_REG        0x32
#define MPU6500_I2C_SLV4_DO         0x33  // Data to be written to I2C Slave 4 if enabled
#define MPU6500_I2C_SLV4_CTRL       0x34
#define MPU6500_I2C_SLV4_DI         0x35  // Data read from I2C Slave 4
#define MPU6500_I2C_MST_STATUS      0x36 
#define MPU6500_INT_PIN_CFG         0x37  //�ж�����
#define MPU6500_INT_ENABLE          0x38 
#define MPU6500_INT_STATUS          0x3A
#define MPU6500_ACCEL_XOUT_H        0x3B  //���ٶȼ�����
#define MPU6500_ACCEL_XOUT_L        0x3C
#define MPU6500_ACCEL_YOUT_H        0x3D
#define MPU6500_ACCEL_YOUT_L        0x3E
#define MPU6500_ACCEL_ZOUT_H        0x3F
#define MPU6500_ACCEL_ZOUT_L        0x40
#define MPU6500_TEMP_OUT_H          0x41  //�¶ȼ�����
#define MPU6500_TEMP_OUT_L          0x42
#define MPU6500_GYRO_XOUT_H         0x43  //����������
#define MPU6500_GYRO_XOUT_L         0x44
#define MPU6500_GYRO_YOUT_H         0x45
#define MPU6500_GYRO_YOUT_L         0x46
#define MPU6500_GYRO_ZOUT_H         0x47
#define MPU6500_GYRO_ZOUT_L         0x48
#define MPU6500_EXT_SENS_DATA_00    0x49  //�洢slave0������
#define MPU6500_EXT_SENS_DATA_01    0x4A
#define MPU6500_EXT_SENS_DATA_02    0x4B
#define MPU6500_EXT_SENS_DATA_03    0x4C
#define MPU6500_EXT_SENS_DATA_04    0x4D
#define MPU6500_EXT_SENS_DATA_05    0x4E
#define MPU6500_EXT_SENS_DATA_06    0x4F
#define MPU6500_EXT_SENS_DATA_07    0x50
#define MPU6500_EXT_SENS_DATA_08    0x51
#define MPU6500_EXT_SENS_DATA_09    0x52
#define MPU6500_EXT_SENS_DATA_10    0x53
#define MPU6500_EXT_SENS_DATA_11    0x54
#define MPU6500_EXT_SENS_DATA_12    0x55
#define MPU6500_EXT_SENS_DATA_13    0x56
#define MPU6500_EXT_SENS_DATA_14    0x57
#define MPU6500_EXT_SENS_DATA_15    0x58
#define MPU6500_EXT_SENS_DATA_16    0x59
#define MPU6500_EXT_SENS_DATA_17    0x5A
#define MPU6500_EXT_SENS_DATA_18    0x5B
#define MPU6500_EXT_SENS_DATA_19    0x5C
#define MPU6500_EXT_SENS_DATA_20    0x5D
#define MPU6500_EXT_SENS_DATA_21    0x5E
#define MPU6500_EXT_SENS_DATA_22    0x5F
#define MPU6500_EXT_SENS_DATA_23    0x60
#define MPU6500_I2C_SLV0_DO         0x63  //Data out when slave 0 is set to write
#define MPU6500_I2C_SLV1_DO         0x64
#define MPU6500_I2C_SLV2_DO         0x65
#define MPU6500_I2C_SLV3_DO         0x66
#define MPU6500_I2C_MST_DELAY_CTRL  0x67  //��ʱ
#define MPU6500_SIGNAL_PATH_RESET   0x68  //������������ݣ�����ֵ��0x07(�����ǣ����ٶȼ�)
#define MPU6500_MOT_DETECT_CTRL     0x69
#define MPU6500_USER_CTRL           0x6A  //�û����õ�Ϊ0X10ʱʹ��SPIģʽ,0x11(SPIģʽ��ͬʱ�������������������Ĵ�������)
#define MPU6500_PWR_MGMT_1          0x6B  //��Դ����1������ֵ��0x00(��������) 0x80(Reset Device) 0x01��Clock Source��
#define MPU6500_PWR_MGMT_2          0x6C  //��Դ����2������ֵ��0x00(Enable Acc & Gyro)
#define MPU6500_FIFO_COUNTH         0x72
#define MPU6500_FIFO_COUNTL         0x73
#define MPU6500_FIFO_R_W            0x74
#define MPU6500_WHO_AM_I            0x75	//ID �Ĵ���Ĭ��ֵ��0x71��ֻ��
#define MPU6500_XA_OFFSET_H         0x77
#define MPU6500_XA_OFFSET_L         0x78
#define MPU6500_YA_OFFSET_H         0x7A
#define MPU6500_YA_OFFSET_L         0x7B
#define MPU6500_ZA_OFFSET_H         0x7D
#define MPU6500_ZA_OFFSET_L         0x7E
/* ---- AK8963 Reg In MPU6500 ----------------------------------------------- */
#define AK8963_I2C_ADDR             0x0C   //0C
#define AK8963_POWER_DOWN   0x10
#define AK8963_FUSE_ROM_ACCESS 0x1F
#define AK8963_CNTL2_SRST 0x01
#define AK8963_CONTINUOUS_MEASUREMENT 0x16 //MODE2ˢ������ 100Hz
// Read-only Reg
#define AK8963_WIA                  0x00  //ֻ�� 0x48	
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02  //Data Status1
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09  //Data reading end register & check Magnetic sensor overflow occurred
// Write/Read Reg
#define AK8963_CNTL1                0x0A  // 0x16  100Hz 16bit Output continuous measurement
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg
#define AK8963_ASAX                 0x10  //X-axis sensitivity adjustment value
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

#endif
