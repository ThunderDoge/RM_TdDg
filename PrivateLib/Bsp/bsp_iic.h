#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "iic.h"

#define Bsp_IIC_SCL_HIGH			HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_SET)
#define Bsp_IIC_SCL_LOW				HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_RESET)
#define Bsp_IIC_SDA_HIGH			HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_SET)
#define BSP_IIC_SDA_LOW				HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_RESET)
#define Bsp_IIC_SDA_READ			HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port,IIC_SDA_Pin)

void bsp_IIC_Delay(void);
void bsp_IIC_Init(void);
void bsp_IIC_Start(void);
void bsp_IIC_Stop(void);
void bsp_IIC_Ack(void);
void bsp_IIC_NAck(void);
uint8_t bsp_IIC_WaitAck(void);
void bsp_IIC_SendByte(uint8_t message);
uint8_t bsp_IIC_ReadByte(uint8_t Ack);

#endif
