/** 
* @brief    视觉串口协议文件
* @details  32与小主机的通讯处理
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020电控
* @par 日志
    2019/12/8   将Evan-GH个人串口版本1.2合并到公共代码库串口v2.1.0
    2019/12/8   ThunderDoge依据自己的喜好将串口库更改为v2.1.1   增加了部分功能。修改了一些写法
*/
//与视觉的通信协议参见《RM2020基本视觉协议 v2.0》 by Evan-GH

#ifndef __BSP_VISION_HPP
#define __BSP_VISION_HPP

#include "stm32f4xx.h"
#include <string.h>

//外设相关宏定义,移植时如果修改了外设请在这里修改
#define BSP_VISION_UART huart6
//视觉串口接收缓存的数组大小，有需要请在这里修改,使用空闲中断要求这个数至少要大于18
#define BSP_VISION_BUFFER_SIZE 60

//数据帧关键位置
enum __bsp_vision_Keywords
{
    Frame_header = 0,  //帧头
    Function_word = 1, //功能字
    Sum_check = 16,    //和校验
    Frame_end = 17     //帧尾
};

//数据帧功能字
enum __bsp_vision_Functionwords
{
    //帧头帧尾
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
    //视觉发给电控的
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01, //控制云台相对角度
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02, //控制云台绝对角度
    CMD_SHOOT = 0x03,                   //射击指令
    CMD_CHASSIS_CONTROL = 0X04,         //底盘控制
	CMD_CHASSIS_LOACTION_CONTROL = 0X05,	//底盘控制路程
	CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06,	//底盘控制路程带限速

    //电控发给视觉的
    CMD_GET_MCU_STATE = 0x11, //获取电控控制信息
    ROBOT_ERR = 0X12
};

//ROBOT_ERR 的错误码列表
enum __bsp_vision_RobotError
{
    DBUS_OFFLINE = 0X01,
    CAN1_OFFLINE = 0X02,
    CAN2_OFFLINE = 0X03,
    MOTOR_OFFLINE_CNT = 0X04,
    GIMBOL_OFFLINE = 0X05,
    CHASSIS_OFFLINE = 0X06,
    JUDG_OFFLINE = 0X07,
    REBOOTINT = 0X08
};

//视觉传输数据解析结构体
typedef struct __vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; //数据帧功能字
    //底盘数据
    float Vx; //底盘X轴速度
    float Vy; //底盘Y轴速度
	float Px; //底盘X轴路程
	float Py; //底盘Y轴路程
	float SpeedLimit;	//底盘限速
    //云台数据
    float Yaw;          //Yaw轴角度
    float Pitch;        //Pitch轴角度
    uint8_t Cloud_mode; //云台模式
    //射击数据
    uint8_t Shoot_mode; //射击模式
    float Shoot_speed;  //射击速度
    uint8_t Shoot_freq; //射击频率
    //数据标志
    uint8_t Shoot_flag = 0; //视觉射击标志位
    uint8_t Ready_flag = 0; //数据就绪标志位

    //日志系统使用
    uint8_t Error_code = 0;          //错误代码
    int16_t CAN1_motorlist = 0xffff; //CAN1电机列表
    int16_t CAN2_motorlist = 0xffff; //CAN2电机列表
} bsp_vision_data;
//视觉发送数据帧类
struct VisionFrame
{
    VisionFrame(__bsp_vision_Functionwords FW)  //功能字初始化
    {
        Frame[Frame_header] = FRAME_HEADER_DATA;
        Frame[Function_word] = FW;
        Frame[Frame_end] = FRAME_END_DATA;
    };
    //功能字与数据初始化
    VisionFrame(__bsp_vision_Functionwords FW, uint8_t *ptrData) : VisionFrame(FW)
    {
        memcpy(Frame + 2, ptrData, 14);
        pack();
    };
    void pack(void);    //封装用函数
    int8_t load(uint8_t *ptrData, size_t size);   //装载数据
    uint8_t Frame[18];  //数据帧本体

private:
    uint8_t load_iter = 0;  //数组指针
};

extern bsp_vision_data bsp_vision_Send_Data, bsp_vision_Rec_Data; //串口发送，接收数组

void bsp_vision_Init(void);                                   //视觉串口初始化
void bsp_vision_It(void);                                     //视觉串口中断处理
HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword); //视觉传口发送函数
HAL_StatusTypeDef bsp_vision_SendData(VisionFrame& frame);  //视觉串口发送函数，使用数据帧类的话
#endif
