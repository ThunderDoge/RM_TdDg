/**
 * @file: app_vision.hpp 
* @brief    视觉串口协议文件
* @details  32与小主机的通讯处理
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020电控
* @par 日志
    2019/12/8   将Evan-GH个人串口版本1.2合并到公共代码库串口v2.1.0
    2019/12/8   ThunderDoge依据自己的喜好将串口库更改为v2.1.1   增加了部分功能。修改了一些写法
    2020/2/20   ThunderDoge将哨兵专用的串口协议调整写入了该文件，并且改名为app_vision.hpp。
*/
//与视觉的通信协议参见《RM2020基本视觉协议 v2.0》 by Evan-GH

#ifndef __BSP_VISION_HPP
#define __BSP_VISION_HPP

#include "stm32f4xx.h"
#include <string.h>
#include "usart.h"
#include "SentryCloud.hpp"	//因为需要使用_cloud_ctrl_mode所以包含了。没有别的依赖
#include "sentry_ctrl_def.hpp"
#include "sentry_can_commom.hpp"



//外设相关宏定义,移植时如果修改了外设请在这里修改
#define BSP_VISION_UART huart1
//视觉串口接收缓存的数组大小，有需要请在这里修改,使用空闲中断要求这个数至少要大于18
#define BSP_VISION_BUFFER_SIZE 120

///数据帧关键位置
enum __bsp_vision_Keywords
{
    Frame_header = 0,  //帧头
    Function_word = 1, //功能字
    Sum_check = 16,    //和校验
    Frame_end = 17     //帧尾
};

///数据帧功能字
enum __bsp_vision_Commom_Data
{
    //帧头帧尾
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
};

///哨兵视觉信息功能字
enum __bsp_vision_Functionwords
{
    //视觉发给电控的
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //控制云台相对角度
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //控制云台绝对角度
    CMD_SHOOT = 0x03,                        //射击指令
    CMD_CHASSIS_CONTROL = 0X04,              //底盘控制
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //底盘控制路程
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //底盘控制路程带限速
	CMD_GIMBAL_SPEED_CONTROL = 0x10,		 //控制底盘转动速度

    //电控发给视觉的
    CMD_GET_MCU_STATE = 0x11, //获取电控控制信息
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,
};

///视觉传输数据解析结构体
typedef struct __vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; ///<数据帧功能字
    ///底盘数据
    float Vx;         ///<底盘X轴速度
    float Vy;         ///<底盘Y轴速度
    float Px;         ///<底盘X轴路程
    float Py;         ///<底盘Y轴路程
    float SpeedLimit; ///<底盘限速
    uint8_t pillar_flag;
    uint8_t chassis_mode;
    ///云台数据
    float Yaw;          ///<Yaw轴角度
	float YawSoft;
    float Pitch;        ///<Pitch轴角度
    uint8_t Cloud_mode; ///<云台模式
    uint8_t cloud_ctrl_mode;
    ///射击数据
    uint8_t Shoot_mode; ///<射击模式
    float Shoot_speed;  ///<射击速度
    uint8_t Shoot_freq; ///<射击频率
    ///数据标志
    uint32_t UpdateTime;
    ///日志系统使用
    uint8_t Error_code = 0;          ///<错误代码
    int16_t CAN1_motorlist = 0xffff; ///<CAN1电机列表
    int16_t CAN2_motorlist = 0xffff; ///<CAN2电机列表
	//本结构体信息
	uint8_t Ready_flag;	//就绪标志。表示有新数据未处理。
} sentry_vision_data;

///错误代码列表
enum __bsp_vision_RobotError
{
    DBUS_OFFLINE = 0X01,
    CAN1_OFFLINE = 0X02,
    CAN2_OFFLINE = 0X03,
    MOTOR_OFFLINE_CNT = 0X04,
    GIMBOL_OFFLINE = 0X05,
    CHASSIS_OFFLINE = 0X06,
    JUDG_OFFLINE = 0X07,
    REBOOTINT = 0X08,
};


extern sentry_vision_data VisionTx, VisionRx; ///串口发送/接收缓存结构体
extern uint8_t Vision_Txbuffer[18];         ///串口发送暂存数组

void bsp_vision_Init(void);                                   ///视觉串口初始化
void bsp_vision_It(void);                                     ///视觉串口中断处理
// HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword); ///视觉传口发送函数
HAL_StatusTypeDef bsp_vision_SendTxbuffer(uint8_t _Functionword);
void bsp_vision_load_to_txbuffer(uint8_t u8data, int loaction_at_buffdata); ///将数据装入缓存Vision_Txbuffer中
void bsp_vision_load_to_txbuffer(float fdata, int loaction_at_buffdata);    ///将数据装入缓存Vision_Txbuffer中

void CloudVisionTxRoutine(void);  ///主逻辑回调函数。向小主机发送一次VisionTx的全部信息。

#endif
