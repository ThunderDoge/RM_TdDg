/**
 * @file bsp_oled.h
 * @author Evan-GH (751191269@qq.com), Asn
 * @version 1.5
 * @date 2020-03-03
 * @copyright OnePointFive
 */
#ifndef __BSP_OLED_H
#define __BSP_OLED_H
#include "stm32f4xx.h"
#include "bsp_oled_font.h"

#define BSP_OLED_SPI					hspi1
//-----------------OLED����----------------
#define BSP_OLED_CMD  0	//д����
#define BSP_OLED_DATA 1	//д����

#define BSP_OLED_RST_CLR() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_RESET)//OLED_RES
#define BSP_OLED_RST_SET() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_SET)

#define BSP_OLED_DC_CLR()	 HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_RESET)//OLED_DC
#define BSP_OLED_DC_SET()	 HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_SET)

#define BSP_OLED_CS_CLR()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin,GPIO_PIN_RESET)//OLED_CS
#define BSP_OLED_CS_SET()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin,GPIO_PIN_SET)
typedef enum
{
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION= 0x02,
}bsp_oled_Pen;


//-----------------------------------------
void bsp_oled_Write(uint8_t Data,uint8_t Cmd);//oled spiд����
//OLED�����ú���
void bsp_oled_Refresh_Gram(void);  //�Դ�ˢ��
void bsp_oled_Display_On(void);//������ʾ
void bsp_oled_Set_Pos(uint8_t x, uint8_t y);//��������
void bsp_oled_Display_Off(void);//�ر���ʾ
void bsp_oled_Init(void);//oled��ʼ��
void bsp_oled_Show_Logo(void); //��ʾLogo
void bsp_oled_Clear(void);//����
void bsp_oled_Draw_Point(uint8_t x, uint8_t y, bsp_oled_Pen pen); //����
void bsp_oled_Show_Graphic(uint8_t x, uint8_t y, const picture_t *graphic);  //��ʾ���ͼ��
void bsp_oled_Show_Char(uint8_t col, uint8_t row, uint8_t chr);//��ʾ�ַ�
void bsp_oled_Show_String(uint8_t col, uint8_t row, uint8_t *chr); //��ʾ�ַ���
void bsp_oled_Printf(uint8_t col, uint8_t row, const char *fmt,...);  //OLED��print����
void bsp_oled_Show_Chinese(uint8_t x,uint8_t y,uint8_t no);//��ʾ�����ַ�
void bsp_oled_Display_Fill(uint8_t Fill_Data); //ȫ�����

#endif
