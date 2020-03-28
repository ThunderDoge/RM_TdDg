/**
* @file bsp_oled.c
* @brief OLED板级支持包
* @author Evan-GH (751191269@qq.com), Asn
* @version 1.5
* @date 2020-03-03
* @copyright OnePointFive
* @par 日志:
*		v1.0 移植中景园oled例程成功\n
*		v1.1 根据代码规范做了一些修改，移植一部分Evasn-GH的旧oled库到这里来，现在可以显示浮点数了呢\n
*		v1.2 为函数增加了注释\n
*   	v1.3 修改文件依赖关系，现在不再调用app层文件\n
*   	v1.4 重新规范化代码文件编码格式和Doxygen注释\n
*   	v1.5 移植大疆官方开源步兵代码中OLED相关文件的函数和数据\n
*/
#include "bsp_oled.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//OLED相关宏定义
#define SIZE 16
#define XLEVELL		0x00
#define XLEVELH		0x10
#define MAX_COLUMN	128
#define MAX_ROW		64
#define	BRIGHTNESS	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64
#define	ABS(x)   ((x)>0?(x):-(x))
//OLED的显存，OLED左下角为原点
static uint8_t bsp_oled_Gram[8][128];

/**
 * @brief OLED命令/数据写入
 * @param Data 要写入的数据/命令
 * @param Cmd 数据/命令标志
 * @note 0,表示命令;1,表示数据
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
	//HAL_SPI_Transmit_DMA(&BSP_OLED_SPI,&Data,1);//如果开了DMA传输可以试试这句
	BSP_OLED_CS_SET();
	BSP_OLED_DC_SET();
}

/**
 * @brief OLED初始化
 */
void bsp_oled_Init(void)
{
	BSP_OLED_RST_SET();
	HAL_Delay(100);
	BSP_OLED_RST_CLR();
	HAL_Delay(200);
	BSP_OLED_RST_SET();

	bsp_oled_Write(0xAE,BSP_OLED_CMD);  //显示关闭
	bsp_oled_Write(0x40,BSP_OLED_CMD);  //设置起始行地址
	bsp_oled_Write(0x81,BSP_OLED_CMD);  //设置对比度控制寄存器
	bsp_oled_Write(0xFF,BSP_OLED_CMD);  //亮度0x00〜0xff
	bsp_oled_Write(0xA4,BSP_OLED_CMD);  //0xa4，输出跟随RAM内容 0xa5，输出忽略RAM内容
	bsp_oled_Write(0xA6,BSP_OLED_CMD);  //设置普通显示
	bsp_oled_Write(0x20,BSP_OLED_CMD);  //设置内存寻址模式
	bsp_oled_Write(0x00,BSP_OLED_CMD);  //00，水平寻址模式; 01，垂直寻址模式; 10，页面寻址模式（RESET）; 11，无效
	bsp_oled_Write(0x21,BSP_OLED_CMD);
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0x7F,BSP_OLED_CMD);
	bsp_oled_Write(0x22,BSP_OLED_CMD);
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0x07,BSP_OLED_CMD);
	bsp_oled_Write(0x3F,BSP_OLED_CMD);
	bsp_oled_Write(0xC8,BSP_OLED_CMD);  //设置COM输出扫描方向
	bsp_oled_Write(0xA1,BSP_OLED_CMD);  //设置段重新映射0到127
	bsp_oled_Write(0xA8,BSP_OLED_CMD);  //设置多路复用率1到64
	bsp_oled_Write(0x00,BSP_OLED_CMD);
	bsp_oled_Write(0xD3,BSP_OLED_CMD);  //设置显示偏移
	bsp_oled_Write(0x00,BSP_OLED_CMD);  //不偏移
	bsp_oled_Write(0xD5,BSP_OLED_CMD);  //设置显示时钟分频比/振荡器频率
	bsp_oled_Write(0xF0,BSP_OLED_CMD);  //设置分频比
	bsp_oled_Write(0xD9,BSP_OLED_CMD);  //设置预充电时间
	bsp_oled_Write(0x22,BSP_OLED_CMD);
	bsp_oled_Write(0xDA,BSP_OLED_CMD);  //设置com引脚的硬件配置
	bsp_oled_Write(0x12,BSP_OLED_CMD);
	bsp_oled_Write(0xDB,BSP_OLED_CMD);  //设置vcomh
	bsp_oled_Write(0x20,BSP_OLED_CMD);  //0x20,0.77xVcc
	bsp_oled_Write(0x8D,BSP_OLED_CMD);  //使能DC-DC
	bsp_oled_Write(0x14,BSP_OLED_CMD);
	bsp_oled_Write(0xAF,BSP_OLED_CMD);  //打开oled面板

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
 * @brief 设置坐标
 * @param x X轴坐标
 * @param y Y轴坐标
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
 * @brief 开启OLED显示
 */
void bsp_oled_Display_On(void)
{
	bsp_oled_Write(0X8D,BSP_OLED_CMD);  //设置DCDC命令
	bsp_oled_Write(0X14,BSP_OLED_CMD);  //打开DCDC
	bsp_oled_Write(0XAF,BSP_OLED_CMD);  //开始显示
}

/**
 * @brief 关闭OLED显示
 */
void bsp_oled_Display_Off(void)
{
	bsp_oled_Write(0X8D,BSP_OLED_CMD);  //设置DCDC命令
	bsp_oled_Write(0X10,BSP_OLED_CMD);  //关闭DCDC
	bsp_oled_Write(0XAE,BSP_OLED_CMD);  //关闭显示
}

/**
 * @brief OLED刷新整个屏幕的数据
 */
void bsp_oled_Refresh_Gram(void)
{
	bsp_oled_Set_Pos(0,0);
	BSP_OLED_DC_SET();
	BSP_OLED_CS_CLR();
	HAL_SPI_Transmit(&BSP_OLED_SPI,(uint8_t *)&bsp_oled_Gram,1024,10);  //水平寻址下直接写入1024个数据
	BSP_OLED_CS_SET();
	BSP_OLED_DC_SET();
}

/**
 * @brief 显示RM官方Logo
 */
void bsp_oled_Show_Logo(void)
{
    // memcpy(bsp_oled_Gram, RM_LOGO_BMP_TRANS, 1024);  //填充数据
    // bsp_oled_Refresh_Gram();
	bsp_oled_Show_Graphic(1,1,&rm_logo);  //个人感觉这个Logo比上面的那个要好看，自己选择吧
}

/**
 * @brief 全屏填充
 * @param Fill_Data 要填充的数据
 */
void bsp_oled_Display_Fill(uint8_t Fill_Data)
{
	memset(bsp_oled_Gram,Fill_Data,1024);  //填充数据
	bsp_oled_Refresh_Gram();
}

/**
 * @brief 清屏函数
 */
void bsp_oled_Clear(void)
{
	bsp_oled_Display_Fill(0X0);
}

/**
 * @brief 画点函数
 * @param x X轴坐标
 * @param y Y轴坐标
 * @param pen 画笔类型
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
 * @brief 显示相关图形
 * @param x X轴坐标
 * @param y Y轴坐标
 * @param graphic 要显示的图形结构体
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
* @brief  字符显示函数
* @details  在指定位置显示一个字符,包括部分字符
* @param  x:0~127，y:0~63，chr 要显示的字符
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
* @brief  显示字符串
* @details  在屏幕某个位置显示字符串
* @param  x,y 起点坐标	 uint8_t *chr
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
* @brief  显示汉字
* @details  在屏幕某个位置显示汉字
* @param  x,y 起点坐标	 no汉字编号
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

//卸载宏定义
#undef SIZE
#undef XLEVELL
#undef XLEVELH
#undef MAX_COLUMN
#undef MAX_ROW
#undef BRIGHTNESS
#undef X_WIDTH
#undef Y_WIDTH
#undef ABS
