/**
* @file bsp_oled.c
* @brief OLED�弶֧�ְ�
* @author Evan-GH (751191269@qq.com), Asn
* @version 1.5
* @date 2020-03-03
* @copyright OnePointFive
* @par ��־:
*		v1.0 ��ֲ�о�԰oled���̳ɹ�\n
*		v1.1 ���ݴ���淶����һЩ�޸ģ���ֲһ����Evasn-GH�ľ�oled�⵽�����������ڿ�����ʾ����������\n
*		v1.2 Ϊ����������ע��\n
*   	v1.3 �޸��ļ�������ϵ�����ڲ��ٵ���app���ļ�\n
*   	v1.4 ���¹淶�������ļ������ʽ��Doxygenע��\n
*   	v1.5 ��ֲ�󽮹ٷ���Դ����������OLED����ļ��ĺ���������\n
*/
#include "bsp_oled.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//OLED��غ궨��
#define SIZE 16
#define XLEVELL		0x00
#define XLEVELH		0x10
#define MAX_COLUMN	128
#define MAX_ROW		64
#define	BRIGHTNESS	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64
#define	ABS(x)   ((x)>0?(x):-(x))
//OLED���Դ棬OLED���½�Ϊԭ��
static uint8_t bsp_oled_Gram[8][128];

/**
 * @brief OLED����/����д��
 * @param Data Ҫд�������/����
 * @param Cmd ����/�����־
 * @note 0,��ʾ����;1,��ʾ����
 */
void bsp_oled_Write(uint8_t Data,uint8_t Cmd)
{
	if(Cmd==1)
	{
	  BSP_OLED_DC_SET();
	}
	else
	{
	  BSP_OLED_DC_CLR();
	}
	BSP_OLED_CS_CLR();
	HAL_SPI_Transmit(&BSP_OLED_SPI,&Data,1,0xff);
	//HAL_SPI_Transmit_DMA(&BSP_OLED_SPI,&Data,1);//�������DMA��������������
	BSP_OLED_CS_SET();
	BSP_OLED_DC_SET();
}

/**
 * @brief OLED��ʼ��
 */
void bsp_oled_Init(void)
{
	BSP_OLED_RST_SET();
	HAL_Delay(100);
	BSP_OLED_RST_CLR();
	HAL_Delay(200);
	BSP_OLED_RST_SET();

	bsp_oled_Write(0xAE,BSP_OLED_CMD);  //��ʾ�ر�
	bsp_oled_Write(0x40,BSP_OLED_CMD);  //������ʼ�е�ַ
	bsp_oled_Write(0x81,BSP_OLED_CMD);  //���öԱȶȿ��ƼĴ���
	bsp_oled_Write(0xFF,BSP_OLED_CMD);  //����0x00?0xff
	bsp_oled_Write(0xA4,BSP_OLED_CMD);  //0xa4���������RAM���� 0xa5���������RAM����
	bsp_oled_Write(0xA6,BSP_OLED_CMD);  //������ͨ��ʾ
	bsp_oled_Write(0x20,BSP_OLED_CMD);  //�����ڴ�Ѱַģʽ
	bsp_oled_Write(0x00,BSP_OLED_CMD);  //00��ˮƽѰַģʽ; 01����ֱѰַģʽ; 10��ҳ��Ѱַģʽ��RESET��; 11����Ч
	bsp_oled_Write(0x21,BSP_OLED_CMD);
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0x7F,BSP_OLED_CMD);
	bsp_oled_Write(0x22,BSP_OLED_CMD);
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0x07,BSP_OLED_CMD);
	bsp_oled_Write(0x3F,BSP_OLED_CMD);
	bsp_oled_Write(0xC8,BSP_OLED_CMD);  //����COM���ɨ�跽��
	bsp_oled_Write(0xA1,BSP_OLED_CMD);  //���ö�����ӳ��0��127
	bsp_oled_Write(0xA8,BSP_OLED_CMD);  //���ö�·������1��64
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0xD3,BSP_OLED_CMD);  //������ʾƫ��
	bsp_oled_Write(0x00,BSP_OLED_CMD);  //��ƫ��
	bsp_oled_Write(0xD5,BSP_OLED_CMD);  //������ʾʱ�ӷ�Ƶ��/����Ƶ��
	bsp_oled_Write(0xF0,BSP_OLED_CMD);  //���÷�Ƶ��
	bsp_oled_Write(0xD9,BSP_OLED_CMD);  //����Ԥ���ʱ��
	bsp_oled_Write(0x22,BSP_OLED_CMD);
	bsp_oled_Write(0xDA,BSP_OLED_CMD);  //����com���ŵ�Ӳ������
	bsp_oled_Write(0x12,BSP_OLED_CMD);
	bsp_oled_Write(0xDB,BSP_OLED_CMD);  //����vcomh
	bsp_oled_Write(0x20,BSP_OLED_CMD);  //0x20,0.77xVcc
	bsp_oled_Write(0x8D,BSP_OLED_CMD);  //ʹ��DC-DC
	bsp_oled_Write(0x14,BSP_OLED_CMD);
	bsp_oled_Write(0xAF,BSP_OLED_CMD);  //��oled���

	bsp_oled_Clear();
	bsp_oled_Show_Logo();
	//uint8_t show_col, show_row;
	//bsp_oled_Show_Graphic(1,1,&offline_icon1);
	// bsp_oled_Show_Graphic(0, 1, &battery_box);
	// bsp_oled_Show_Graphic(100,1, &wifi_signal[0]);
	// for(int i = 0; i < 9; i++)
	// {
	// 	show_col = ((i-1) * 32) % 128;
	// 	show_row = 15 + (i-1) / 4 * 12;
	// 	bsp_oled_Printf(show_col, show_row, "%c", 'M');
	// 	bsp_oled_Printf(show_col + 6, show_row, "%c", '0'+i);
	// 	bsp_oled_Show_Graphic(show_col + 12, show_row, &check_box[0]);
	// }
	bsp_oled_Refresh_Gram();
}

/**
 * @brief ��������
 * @param x X������
 * @param y Y������
 */
void bsp_oled_Set_Pos(uint8_t x, uint8_t y)
{
	x &= 0x7f;
	y &= 0x07;

	bsp_oled_Write(0x21,BSP_OLED_CMD);
	bsp_oled_Write(0X00+x,BSP_OLED_CMD);
	bsp_oled_Write(0X7F,BSP_OLED_CMD);

	bsp_oled_Write(0x22,BSP_OLED_CMD);
	bsp_oled_Write(0X00+y,BSP_OLED_CMD);
	bsp_oled_Write(0X07,BSP_OLED_CMD);
}

/**
 * @brief ����OLED��ʾ
 */
void bsp_oled_Display_On(void)
{
	bsp_oled_Write(0X8D,BSP_OLED_CMD);  //����DCDC����
	bsp_oled_Write(0X14,BSP_OLED_CMD);  //��DCDC
	bsp_oled_Write(0XAF,BSP_OLED_CMD);  //��ʼ��ʾ
}

/**
 * @brief �ر�OLED��ʾ
 */
void bsp_oled_Display_Off(void)
{
	bsp_oled_Write(0X8D,BSP_OLED_CMD);  //����DCDC����
	bsp_oled_Write(0X10,BSP_OLED_CMD);  //�ر�DCDC
	bsp_oled_Write(0XAE,BSP_OLED_CMD);  //�ر���ʾ
}

/**
 * @brief OLEDˢ��������Ļ������
 */
void bsp_oled_Refresh_Gram(void)
{
	bsp_oled_Set_Pos(0,0);
	BSP_OLED_DC_SET();
	BSP_OLED_CS_CLR();
	HAL_SPI_Transmit(&BSP_OLED_SPI,(uint8_t *)&bsp_oled_Gram,1024,10);  //ˮƽѰַ��ֱ��д��1024������
	BSP_OLED_CS_SET();
	BSP_OLED_DC_SET();
}

/**
 * @brief ��ʾRM�ٷ�Logo
 */
void bsp_oled_Show_Logo(void)
{
    // memcpy(bsp_oled_Gram, RM_LOGO_BMP_TRANS, 1024);  //�������
    // bsp_oled_Refresh_Gram();
	bsp_oled_Show_Graphic(1,1,&rm_logo);  //���˸о����Logo��������Ǹ�Ҫ�ÿ����Լ�ѡ���
}

/**
 * @brief ȫ�����
 * @param Fill_Data Ҫ��������
 */
void bsp_oled_Display_Fill(uint8_t Fill_Data)
{
	memset(bsp_oled_Gram,Fill_Data,1024);  //�������
	bsp_oled_Refresh_Gram();
}

/**
 * @brief ��������
 */
void bsp_oled_Clear(void)
{
	bsp_oled_Display_Fill(0X0);
}

/**
 * @brief ���㺯��
 * @param x X������
 * @param y Y������
 * @param pen ��������
 */
void bsp_oled_Draw_Point(uint8_t x, uint8_t y, bsp_oled_Pen pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ( (x > (X_WIDTH - 1)) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        bsp_oled_Gram[page][x] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        bsp_oled_Gram[page][x] ^= 1 << row;
    }
    else
    {
        bsp_oled_Gram[page][x] &= ~(1 << row);
    }
}

/**
 * @brief ��ʾ���ͼ��
 * @param x X������
 * @param y Y������
 * @param graphic Ҫ��ʾ��ͼ�νṹ��
 */
void bsp_oled_Show_Graphic(uint8_t x, uint8_t y, const picture_t *graphic)
{
    uint8_t col, row;
    uint8_t temp_char, t;
    uint16_t i = 0;

    for(col = 0; col < graphic->length; col++)
    {
        for(row = 0; row < graphic->width; )
        {
            temp_char = graphic->data[i];
            i++;
            for(t = 0; t < 8; t++)
            {
                if(temp_char & 0x80)
                {
                    bsp_oled_Draw_Point(x + col, y + row,PEN_WRITE);
                }
                else
                {
                    bsp_oled_Draw_Point(x + col, y + row,PEN_CLEAR);
                }
                temp_char <<= 1;
                row++;
                if(row == graphic->width)
                {
                    break;
                }
            }
        }
    }
}

/**
* @brief  �ַ���ʾ����
* @details  ��ָ��λ����ʾһ���ַ�,���������ַ�
* @param  x:0~127��y:0~63��chr Ҫ��ʾ���ַ�
* @retval  NULL
*/
void bsp_oled_Show_Char(uint8_t col, uint8_t row, uint8_t chr)
{
	uint8_t x = col;
    uint8_t y = row ;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                bsp_oled_Draw_Point(x, y, PEN_WRITE);
            else
                bsp_oled_Draw_Point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
* @brief  ��ʾ�ַ���
* @details  ����Ļĳ��λ����ʾ�ַ���
* @param  x,y �������	 uint8_t *chr
* @retval  NULL
*/
void bsp_oled_Show_String(uint8_t col, uint8_t row, uint8_t *chr)
{
	uint8_t n =0;

    while (chr[n] != '\0')
    {
        bsp_oled_Show_Char(col, row, chr[n]);
        col+=6;

        if (col > X_WIDTH - 6)
        {
            col = 0;
            row += 12;
        }
        n++;
    }
}

void bsp_oled_Printf(uint8_t col, uint8_t row, const char *fmt,...)
{
    static uint8_t LCD_BUF[22] = {0};
    static va_list ap;
    uint16_t remain_size = 0;

    va_start(ap, fmt);

    remain_size = vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

	LCD_BUF[remain_size] = '\0';

    bsp_oled_Show_String(col, row, LCD_BUF);
}

/**
* @brief  ��ʾ����
* @details  ����Ļĳ��λ����ʾ����
* @param  x,y �������	 no���ֱ��
* @retval  NULL
*/
void bsp_oled_Show_Chinese(uint8_t x,uint8_t y,uint8_t no)
{
	uint8_t t,adder=0;
	bsp_oled_Set_Pos(x,y);
    for(t=0;t<16;t++)
	{
		bsp_oled_Write(Chinese[2*no][t],BSP_OLED_DATA);
		adder+=1;
    }
		bsp_oled_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
	{
		bsp_oled_Write(Chinese[2*no+1][t],BSP_OLED_DATA);
		adder+=1;
	}
}

//ж�غ궨��
#undef SIZE
#undef XLEVELL
#undef XLEVELH
#undef MAX_COLUMN
#undef MAX_ROW
#undef BRIGHTNESS
#undef X_WIDTH
#undef Y_WIDTH
#undef ABS
