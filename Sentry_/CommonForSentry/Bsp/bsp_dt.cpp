/**
 * @file bsp_dt.cpp
 * @brief 数据传输板级支持包
 * @author Evan-GH (751191269@qq.com)
 * @version 1.4
 * @date 2020-02-28
 * @copyright OnePointFive
 * @par 日志:
 *   V1.0 创建此文件管理对外发送数据的相关函数和数据\n
 *   V1.1 重新规范化代码文件编码格式和Doxygen注释\n
 *   V1.2 完善发送和接收处理的相关函数\n
 *   V1.3 初步添加对匿名上位机的数据传输支持\n
 *   v1.4 完成对于匿名上位机V7版本新协议的适配\n
 */
#include "bsp_dt.hpp"
#include "usart.h"
#include "app_imu.h"
#include <string.h>
#include "bsp_dbus.h"

//使用匿名上位机传输数据需要的宏定义，用于拆分一个整型数的高低位
#define BYTE0(Data)      (*(char *)(&Data))
#define BYTE1(Data)      (*((char *)(&Data)+1))
#define BYTE2(Data)      (*((char *)(&Data)+2))
#define BYTE3(Data)      (*((char *)(&Data)+3))

bsp_dt_data bsp_dt_Send_Data; //!< 存储对外发送信息的结构体
static uint8_t DT_Rxbuffer[BSP_DT_BUFFER_SIZE]={0}; //!< 串口接收数据缓存数组
int32_t bsp_dt_ParList[166];  //参数列表

static uint8_t DT_Txbuffer[16]={0};  //串口发送用数组
/**
 * @brief 使用自己的上位机解析数据的串口发送函数
 * @return HAL_StatusTypeDef
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_Sendmessage(void)
{
	int16_t _check_sum = 0; //和校验用变量
	memset(DT_Txbuffer,0,16); //发送之前先清空一次
	DT_Txbuffer[0] = 0xff; //帧头
	DT_Txbuffer[1] = bsp_dt_Send_Data.Function_word;  //功能字
	DT_Txbuffer[15] = 0x0d; //帧尾

	memcpy(DT_Txbuffer+2,&bsp_dt_Send_Data.Data_1,4);
	memcpy(DT_Txbuffer+6,&bsp_dt_Send_Data.Data_2,4);
	memcpy(DT_Txbuffer+10,&bsp_dt_Send_Data.Data_3,4);

	for(int i=0; i<16; i++) //和校验
	{
		if(i!=14)	_check_sum+=DT_Txbuffer[i];  //除了和校验那一位其他位上的数值加和
	}
	_check_sum = _check_sum & 0xff;
	DT_Txbuffer[14] = _check_sum;

	return HAL_UART_Transmit_DMA(&BSP_DT_UART,DT_Txbuffer,16);
}

static uint8_t ANO_DT_Txbuffer[50] = {0};  //串口发送用缓存，匿名上位机用
/**
 * @brief 向匿名上位机发送用户自定义数据
 * @param _Frame_ID 帧ID
 * @param Data1 数据1
 * @param Data2 数据2
 * @param Data3 数据3
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_UserData(uint8_t _Frame_ID, int32_t Data1, int32_t Data2, int32_t Data3)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = _Frame_ID;  //发送用户自定义数据
	ANO_DT_Txbuffer[cnt++] = 12;  //数据长度

	int32_t temp = Data1;  //填充Data1
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);
	temp = Data2;  //填充Data2
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);
	temp = Data3;  //填充Data3
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送传感器数据1
 * @param Acc_x X轴加速度
 * @param Acc_y Y轴加速度
 * @param Acc_z Z轴加速度
 * @param Gyro_x X轴角速度
 * @param Gyro_y Y轴角速度
 * @param Gyro_z Z轴角速度
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData1(int16_t Acc_x, int16_t Acc_y, int16_t Acc_z, int16_t Gyro_x, int16_t Gyro_y, int16_t Gyro_z)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0X01;  //发送传感器数据1
	ANO_DT_Txbuffer[cnt++] = 13;  //数据长度

	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_x);  //填充X轴加速度
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_y);  //填充Y轴加速度
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_z);  //填充Z轴加速度
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_z);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_x);  //填充X轴陀螺仪数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_y);  //填充Y轴陀螺仪数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_z);  //填充Z轴陀螺仪数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_z);
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 向匿名上位机发送传感器数据2，温度扩大十倍
 * @param Mag_x X轴磁力计数据
 * @param Mag_y Y轴磁力计数据
 * @param Mag_z Z轴磁力计数据
 * @param Alt_Bar 气压计高度数据，单位CM
 * @param Temp 温度数据
 * @param Bar_STA 气压计状态
 * @param Mag_STA 磁力计状态
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData2(int16_t Mag_x, int16_t Mag_y, int16_t Mag_z, float Alt_Bar, float Temp, uint8_t Bar_STA, uint8_t Mag_STA)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0X02;  //发送传感器数据2
	ANO_DT_Txbuffer[cnt++] = 14;  //数据长度

	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_x);  //填充X轴磁力计数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_y);  //填充Y轴磁力计数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_z);  //填充Z轴磁力计数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_z);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Alt_Bar);  //填充气压计高度数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Alt_Bar);
	ANO_DT_Txbuffer[cnt++] = BYTE2(Alt_Bar);
	ANO_DT_Txbuffer[cnt++] = BYTE3(Alt_Bar);
	int16_t temp = Temp * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充温度数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Bar_STA);  //填充状态数据
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_STA);

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送欧拉角数据，数据扩大100倍
 * @param Pitch 俯仰角
 * @param Roll 翻滚角
 * @param Yaw 偏航角
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_EulerAngle(float Pitch, float Roll, float Yaw)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xAF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0X03;  //发送欧拉角
	ANO_DT_Txbuffer[cnt++] = 0x07;  //数据长度

	int16_t temp;
	//归一化角度
	if(Roll>0 && Roll<180)
		temp = (180-Roll)*100;
	if(Roll<0 && Roll>-180)
		temp = (-180-Roll)*100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充翻滚角
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Pitch * 100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充俯仰角
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Yaw * 100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充偏航角
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送遥控器数据
 * @param CH0 通道0数据
 * @param CH1 通道1数据
 * @param CH2 通道2数据
 * @param CH3 通道3数据
 * @param S1 拨码开关1
 * @param S2 拨码开关2
 * @param Dial 拨轮数据
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Remote(int16_t CH0, int16_t CH1, int16_t CH2, int16_t CH3, int16_t S1, int16_t S2, int16_t Dial)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0X40;  //发送遥控器数据
	ANO_DT_Txbuffer[cnt++] = 20;  //数据长度

	ANO_DT_Txbuffer[cnt++] = BYTE0(CH0);  //填充通道0数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH0);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH1);  //填充通道1数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH1);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH2);  //填充通道2数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH2);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH3);  //填充通道3数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH3);
	ANO_DT_Txbuffer[cnt++] = BYTE0(S1);  //填充拨码开关1数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(S1);
	ANO_DT_Txbuffer[cnt++] = BYTE0(S2);  //填充拨码开关2数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(S2);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Dial);  //填充拨轮数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(Dial);

	//后面的数据没有就补0
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送电压电流数据，数据扩大十倍
 * @param Volatile 电压数据
 * @param Current 电流数据
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_VolatileCurrent(float Volatile, float Current)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0X0D;  //发送遥控器数据
	ANO_DT_Txbuffer[cnt++] = 4;  //数据长度

	uint16_t temp = Volatile * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充电压数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Current * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //填充电流数据
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送一个字符串
 * @param Color 字符串颜色
 * @param String 发送的字符串
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_LogString(uint8_t Color, uint8_t* String)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0XA0;  //发送传感器数据
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = Color;  //设置颜色
	uint8_t i = 0;
	while(*(String+i) != '\0')
	{
		ANO_DT_Txbuffer[cnt++] = *(String+i++);  //填充字符串到帧中
		if(cnt > 50) break;  //帧长度最多50
	}
	ANO_DT_Txbuffer[3] = cnt - 4;
	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 向匿名上位机发送log信息，数字＋字符串
 * @param Num 数值
 * @param String 字符串信息
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_NumString(int32_t Num,uint8_t *String)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0XA0;  //发送传感器数据
	ANO_DT_Txbuffer[cnt++] = 0;

	ANO_DT_Txbuffer[cnt++] = BYTE0(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE1(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE2(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE3(Num);

	uint8_t i = 0;
	while(*(String+i) != '\0')
	{
		ANO_DT_Txbuffer[cnt++] = *(String+i++);  //填充字符串到帧中
		if(cnt > 50) break;  //帧长度最多50
	}
	ANO_DT_Txbuffer[3] = cnt - 4;
	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 发送校验信息到上位机
 * @param _Sc 收到的和校验
 * @param _Ac 收到的附加校验
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Check(uint8_t Frame_ID, uint8_t _Sc, uint8_t _Ac)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0x00;  //发送反馈帧
	ANO_DT_Txbuffer[cnt++] = 3;  //数据长度
	ANO_DT_Txbuffer[cnt++] = Frame_ID;
	ANO_DT_Txbuffer[cnt++] = _Sc;  //填充接收到的校验信息
	ANO_DT_Txbuffer[cnt++] = _Ac;

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief 对匿名上位机发送被查询的参数值，PID参数扩大1000倍上传
 * @param _ParaID
 * @return HAL_StatusTypeDef 发送结果
 * @retval HAL_OK 发送成功
 * @retval HAL_ERROR 发送失败
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Parameter(uint16_t _ParaID)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //帧头
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //目标地址
	ANO_DT_Txbuffer[cnt++] = 0xE2;  //发送反馈帧
	ANO_DT_Txbuffer[cnt++] = 6;  //数据长度

	ANO_DT_Txbuffer[cnt++] = BYTE0(_ParaID);
	ANO_DT_Txbuffer[cnt++] = BYTE1(_ParaID);

	int32_t temp = bsp_dt_ParList[_ParaID];
	if(temp >= 11 && temp <=64)  //PID参数列表发送时候要把对应浮点数据扩大1000倍
	{

	}
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);

	uint8_t sc=0,ac=0;  //校验
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

static void bsp_dt_Ano_Data_Analysis(void)
{
	uint8_t sc=0,ac=0;  //校验用变量
	uint16_t ParID;

	if (DT_Rxbuffer[0] == 0xAA && (DT_Rxbuffer[1] == 0xFF || DT_Rxbuffer[1] == 0x05))  //检测帧的头两个数据
	{
		switch (DT_Rxbuffer[2])
		{
			case 0xE0:  //命令操作
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //校验不通过，返回
					return;
				bsp_dt_ANO_Send_Check(0XE0,sc,ac);  //发送回馈指令
				if(DT_Rxbuffer[4] == 0x02)  //系统重启命令，要在发送反馈报文之后执行，否则会反复重启
				{
					__set_PRIMASK(1);  //关闭中断
					NVIC_SystemReset();  //请求单片机重启
				}
			break;
			case 0xE1:  //读取参数
				ParID = DT_Rxbuffer[5]<<8 | DT_Rxbuffer[4];  //获取参数ID
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //校验不通过，返回
					return;
				bsp_dt_ANO_Send_Parameter(ParID);
			break;
			case 0xE2:  //写入参数
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				ParID = DT_Rxbuffer[5]<<8 | DT_Rxbuffer[4];
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //校验不通过，返回
					return;
				bsp_dt_ANO_Send_Check(0XE2,sc,ac);
				if(ParID >= 11 && ParID <=64)  //更新PID参数列表时候要缩小1000倍再更新到对应数据里
					bsp_dt_ParList[ParID] = (int32_t)(DT_Rxbuffer[6] | DT_Rxbuffer[7]<<8 | DT_Rxbuffer[8]<<16 | DT_Rxbuffer[9]<<24);
				else
					bsp_dt_ParList[ParID] = (int32_t)(DT_Rxbuffer[6] | DT_Rxbuffer[7]<<8 | DT_Rxbuffer[8]<<16 | DT_Rxbuffer[9]<<24);
			break;
			default:
			break;
		}
	}
	else
	{
		return;  //帧头数据不对，返回
	}
}

/**
 * @brief 数据传输初始化
 */
void bsp_dt_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&BSP_DT_UART); //清除空闲中断位
	__HAL_UART_ENABLE_IT(&BSP_DT_UART,UART_IT_IDLE); //使能DMA接收空闲中断
	HAL_UART_Receive_DMA(&BSP_DT_UART, (uint8_t*)DT_Rxbuffer, BSP_DT_BUFFER_SIZE); //开始DMA接收

	//初始化版本参数，模拟为拓空者飞控，否则一部分功能没法用
	bsp_dt_ParList[BSP_DT_PAR_DEVICE_NAME] = INFANTRY_NAME;
	bsp_dt_ParList[BSP_DT_PAR_HWTYPE] = INFANTRY_HWTYPE;
	bsp_dt_ParList[BSP_DT_PAR_HWVER] = INFANTRY_HWVER;
	bsp_dt_ParList[BSP_DT_PAR_SWVER] = INFANTRY_SWVER;
	bsp_dt_ParList[BSP_DT_PAR_BLVER] = INFANTRY_BLVER;
}

/**
 * @brief 数据传输中断处理
 */
void bsp_dt_It(void)
{
	if(__HAL_UART_GET_FLAG(&BSP_DT_UART,UART_FLAG_IDLE) != RESET)	//如果产生了空闲中断
	{
		HAL_UART_DMAStop(&BSP_DT_UART); //关闭DMA
		bsp_dt_Ano_Data_Analysis();
		memset(DT_Rxbuffer,0,BSP_DT_BUFFER_SIZE); //解析完成，数据清0
		__HAL_UART_CLEAR_IDLEFLAG(&BSP_DT_UART); //清除空闲中断标志位
		HAL_UART_DMAResume(&BSP_DT_UART);         //重新打开DMA，这句记得加
		HAL_UART_Receive_DMA(&BSP_DT_UART, (uint8_t*)DT_Rxbuffer, BSP_DT_BUFFER_SIZE);//重新开启DMA接收传输
	}
}

/**
 * @brief 向匿名上位机发送信息的句柄函数
 * 此函数1MS调用一次，内部通过状态机来实现不同频率的数据传送
 */
void bsp_dt_ANO_Send_Handle(void)
{
	static uint32_t cnt = 0;
	static uint32_t senser_cnt 	= 10;
	static uint32_t senser2_cnt 	= 50;
	static uint32_t user_cnt 	= 10;
	static uint32_t status_cnt 	= 15;
	static uint32_t rcdata_cnt 	= 20;
	//static uint32_t motopwm_cnt	= 20;
	static uint32_t power_cnt	= 1;
	//static uint32_t speed_cnt   	= 50;
	//static uint32_t location_cnt = 500;

	if((cnt % status_cnt) == (status_cnt-1))
		bsp_dt_ANO_Send_EulerAngle(app_imu_data.Pitch,app_imu_data.Roll,app_imu_data.Yaw);

	if((cnt % senser_cnt) == (senser_cnt-1))
		bsp_dt_ANO_Send_SensorData1(app_imu_data.original.Accel[0],app_imu_data.LPF.Accel[0],app_imu_data.original.Accel[2],
							app_imu_data.original.Gyro[0],app_imu_data.original.Gyro[1],app_imu_data.original.Gyro[2]);

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		bsp_dt_ANO_Send_SensorData2(0,0,0,0,app_imu_data.unitized.MPU_Temp,0,0);

	if((cnt % power_cnt) == (power_cnt-1))
		bsp_dt_ANO_Send_VolatileCurrent(24.0,1.55);

	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		bsp_dt_ANO_Send_Remote(bsp_dbus_Data.CH_0,bsp_dbus_Data.CH_1,bsp_dbus_Data.CH_2,bsp_dbus_Data.CH_3,bsp_dbus_Data.S1,bsp_dbus_Data.S2,bsp_dbus_Data.Dial);

	if((cnt % user_cnt) == (user_cnt-1))
		bsp_dt_ANO_Send_UserData(0XF1,cnt,-cnt,cnt*cnt);

	if(++cnt>1000) cnt = 0;
}

#undef BYTE0
#undef BYTE1
#undef BYTE2
#undef BYTE3
