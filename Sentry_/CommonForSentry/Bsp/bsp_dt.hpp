/**
 * @file bsp_dt.hpp
 * @author Evan-GH (751191269@qq.com)
 * @version 1.4
 * @date 2020-02-28
 * @copyright OnePointFive
 */
#ifndef  __BSP_DT_HPP
#define  __BSP_DT_HPP
#include "stm32f4xx.h"

#define BSP_DT_UART           huart6
#define BSP_DT_BUFFER_SIZE    50

//匿名上位机读取和写入参数时的序号
#define BSP_DT_PAR_DEVICE_NAME 0
#define BSP_DT_PAR_HWTYPE 1
#define BSP_DT_PAR_HWVER 2
#define BSP_DT_PAR_SWVER 3
#define BSP_DT_PAR_BLVER 4
#define BSP_DT_PAR_INFO_5 5
#define BSP_DT_PAR_INFO_6 6
#define BSP_DT_PAR_INFO_7 7
#define BSP_DT_PAR_INFO_8 8
#define BSP_DT_PAR_INFO_9 9
#define BSP_DT_PAR_INFO_10 10
#define BSP_DT_PAR_PID_1_P 11
#define BSP_DT_PAR_PID_1_I 12
#define BSP_DT_PAR_PID_1_D 13
#define BSP_DT_PAR_PID_2_P 14
#define BSP_DT_PAR_PID_2_I 15
#define BSP_DT_PAR_PID_2_D 16
#define BSP_DT_PAR_PID_3_P 17
#define BSP_DT_PAR_PID_3_I 18
#define BSP_DT_PAR_PID_3_D 19
#define BSP_DT_PAR_PID_4_P 20
#define BSP_DT_PAR_PID_4_I 21
#define BSP_DT_PAR_PID_4_D 22
#define BSP_DT_PAR_PID_5_P 23
#define BSP_DT_PAR_PID_5_I 24
#define BSP_DT_PAR_PID_5_D 25
#define BSP_DT_PAR_PID_6_P 26
#define BSP_DT_PAR_PID_6_I 27
#define BSP_DT_PAR_PID_6_D 28
#define BSP_DT_PAR_PID_7_P 29
#define BSP_DT_PAR_PID_7_I 30
#define BSP_DT_PAR_PID_7_D 31
#define BSP_DT_PAR_PID_8_P 32
#define BSP_DT_PAR_PID_8_I 33
#define BSP_DT_PAR_PID_8_D 34
#define BSP_DT_PAR_PID_9_P 35
#define BSP_DT_PAR_PID_9_I 36
#define BSP_DT_PAR_PID_9_D 37
#define BSP_DT_PAR_PID_10_P 38
#define BSP_DT_PAR_PID_10_I 39
#define BSP_DT_PAR_PID_10_D 40
#define BSP_DT_PAR_PID_11_P 41
#define BSP_DT_PAR_PID_11_I 42
#define BSP_DT_PAR_PID_11_D 43
#define BSP_DT_PAR_PID_12_P 44
#define BSP_DT_PAR_PID_12_I 45
#define BSP_DT_PAR_PID_12_D 46
#define BSP_DT_PAR_PID_13_P 47
#define BSP_DT_PAR_PID_13_I 48
#define BSP_DT_PAR_PID_13_D 49
#define BSP_DT_PAR_PID_14_P 50
#define BSP_DT_PAR_PID_14_I 51
#define BSP_DT_PAR_PID_14_D 52
#define BSP_DT_PAR_PID_15_P 53
#define BSP_DT_PAR_PID_15_I 54
#define BSP_DT_PAR_PID_15_D 55
#define BSP_DT_PAR_PID_16_P 56
#define BSP_DT_PAR_PID_16_I 57
#define BSP_DT_PAR_PID_16_D 58
#define BSP_DT_PAR_PID_17_P 59
#define BSP_DT_PAR_PID_17_I 60
#define BSP_DT_PAR_PID_17_D 61
#define BSP_DT_PAR_PID_18_P 62
#define BSP_DT_PAR_PID_18_I 63
#define BSP_DT_PAR_PID_18_D 64

//步兵的设备信息
#define INFANTRY_NAME 0x05  //!< 模拟成拓空者飞控
#define INFANTRY_HWTYPE 0X05  //!< 步兵硬件种类
#define INFANTRY_HWVER 0x02  //!< 步兵云台板PCB版本
#define INFANTRY_SWVER 0X0A  //!< 步兵云台软件版本
#define INFANTRY_BLVER 0X00  //!< 步兵云台BL版本
/**
 * @brief 监视数据发送结构体
 */
typedef struct __bsp_dt_data
{
	uint8_t Frame_header = 0xff;  //!< 帧头
	uint8_t Frame_end = 0x0d;  //!< 帧尾
    uint8_t Function_word;  //!< 数据帧功能字

    float Data_1;  //!< 要传输的数据1
    float Data_2;  //!< 要传输的数据2
    float Data_3;  //!< 要传输的数据3
}bsp_dt_data;

extern bsp_dt_data bsp_dt_Send_Data; //!< 发送的数据
extern int32_t bsp_dt_ParList[166];

void bsp_dt_Init(void);
void bsp_dt_It(void);
//卑微个人上位机发送用函数
HAL_StatusTypeDef bsp_dt_Sendmessage(void);
//匿名上位机用发送函数
HAL_StatusTypeDef bsp_dt_ANO_Send_UserData(uint8_t _Frame_ID, int32_t Data1, int32_t Data2, int32_t Data3);
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData1(int16_t Acc_x, int16_t Acc_y, int16_t Acc_z, int16_t Gyro_x, int16_t Gyro_y, int16_t Gyro_z);
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData2(int16_t Mag_x, int16_t Mag_y, int16_t Mag_z, float Alt_Bar, float Temp, uint8_t Bar_STA, uint8_t Mag_STA);
HAL_StatusTypeDef bsp_dt_ANO_Send_EulerAngle(float Pitch, float Roll, float Yaw);
HAL_StatusTypeDef bsp_dt_ANO_Send_Remote(int16_t CH0, int16_t CH1, int16_t CH2, int16_t CH3, int16_t S1, int16_t S2, int16_t Dial);
HAL_StatusTypeDef bsp_dt_ANO_Send_VolatileCurrent(float Volatile, float Current);
HAL_StatusTypeDef bsp_dt_ANO_Send_LogString(uint8_t Color, uint8_t* String);
HAL_StatusTypeDef bsp_dt_ANO_Send_NumString(int32_t Num,uint8_t *String);
HAL_StatusTypeDef bsp_dt_ANO_Send_Check(uint8_t Frame_ID, uint8_t _Sc, uint8_t _Ac);
HAL_StatusTypeDef bsp_dt_ANO_Send_BSP_DT_PARameter(uint16_t _BSP_DT_PARaID);
void bsp_dt_ANO_Send_Handle(void);
#endif
