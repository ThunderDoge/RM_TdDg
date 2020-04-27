/**
* @file bsp_ak8975.h
* @brief 
* @author Asn (921576434@qq.com)
* @date 2020.04.10
* @version 1.0
* @copyright Copyright (c) RM2020电控
*/
#ifndef __BSP_AK8975_H
#define __BSP_AK8975_H
#include "stm32f4xx_hal.h"



#define USE_MAG

/////////////////////////////////////
#define AK8975_WIA_REG          0X00 
#define AK8975_INFO_REG         0X01 
#define AK8975_ST1_REG          0X02 
#define AK8975_HXL_REG          0X03 
#define AK8975_HXH_REG          0X04
#define AK8975_HYL_REG          0X05
#define AK8975_HYH_REG          0X06
#define AK8975_HZL_REG          0X07
#define AK8975_HZH_REG          0X08
#define AK8975_ST2_REG          0X09 
#define AK8975_CNTL_REG         0X0A 
#define AK8975_RSV_REG          0X0B
#define AK8975_ASTC_REG         0X0C 
#define AK8975_TS1_REG          0X0D
#define AK8975_TS2_REG          0X0E
#define AK8975_I2CDIS_REG       0X0F 
#define AK8975_ASAX_REG         0X10 
#define AK8975_ASAY_REG         0X11
#define AK8975_ASAZ_REG         0X12  

////////////////////////////////////////////////

#ifdef USE_MAG
extern uint8_t bsp_ak8975_Asa[3];//读取磁力计ASA校正寄存器
void bsp_ak8975_Trig(void);//磁力计触发
uint8_t bsp_ak8975_ReadRegs(uint8_t regAddr,uint8_t *pBuff,uint8_t length);//磁力计读取
HAL_StatusTypeDef bsp_ak8975_Init(void);//磁力计初始化
#endif
#endif
