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
    2019/12/8   ThunderDoge依据自己的喜好将串口库更改为v2.1.1
增加了部分功能。修改了一些写法 2020/2/20
ThunderDoge将哨兵专用的串口协议调整写入了该文件，并且改名为app_vision.hpp。
*/
//与视觉的通信协议参见《RM2020基本视觉协议 v2.0》 by Evan-GH

#ifndef __APP_VISION_HPP
#define __APP_VISION_HPP


#include "sentry_ctrl_def.hpp" //因为需要使用 _cloud_ctrl_mode 所以包含了。
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
//#include "dma.h"
#include <string.h>


//外设相关宏定义,移植时如果修改了外设请在这里修改
#define APP_VISION_UART huart5
//视觉串口接收缓存的数组大小，有需要请在这里修改,使用空闲中断要求这个数至少要大于18
#define APP_VISION_BUFFER_SIZE 120

//使用信号量控制串口 宏定义
//#define APP_VISION_USE_SEMAPHORE	1

#if(APP_VISION_USE_SEMAPHORE)
#include "cmsis_os.h"
#endif

///数据帧关键位置
enum __app_vision_Keywords
{
    Frame_header = 0,  //帧头
    Function_word = 1, //功能字
    Sum_check = 16,    //和校验
    Frame_end = 17     //帧尾
};

///数据帧功能字
enum __app_vision_Commom_Data
{
    //帧头帧尾
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
};

enum __app_vision_pid_id : uint8_t
{
    __cloud_pitch_position_pid = 0U,
    __cloud_pitch_speed_pid ,
    __cloud_yaw_position_pid ,
    __cloud_yaw_speed_pid ,
    __app_vision_pid_id_count,
};

///哨兵视觉信息功能字
enum __app_vision_Functionwords
{
    //视觉发给电控的
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //控制云台相对角度
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //控制云台绝对角度
    CMD_SHOOT = 0x03,                        //射击指令
    CMD_CHASSIS_CONTROL = 0X04,              //底盘控制
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //底盘控制路程
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //底盘控制路程带限速
    CMD_READ_PID = 0X07,
    CMD_WRITE_PID = 0X08,
    CMD_GIMBAL_SPEED_CONTROL = 0x10, //控制底盘转动速度

    //电控发给视觉的
    CMD_GET_MCU_STATE = 0x11, //获取电控控制信息
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,

    JUD_GAME_STATUS = 0x20,
    JUD_SELF_HP = 0x21,
    JUD_BLUE_HP = 0x22,
    JUD_RED_HP = 0x23,
    JUD_GUN_CHASSIS_HEAT = 0x24,
    JUD_SELF_BUFF = 0x25,
    JUD_TAKING_DMG = 0x26,
    JUD_SHOOTING = 0x27,
    JUD_AMMO_LEFT = 0x28,
    // JUD_GAME_EVENT = 0x29,
    JUD_USER = 0x31,
	
	APP_TEST = 0X44,
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
    float Yaw; ///< Yaw轴角度
    float YawSoft;
    float Pitch;        ///< Pitch轴角度
    uint8_t Cloud_mode; ///<云台模式
    uint8_t cloud_ctrl_mode;
    ///射击数据
    uint8_t Shoot_mode; ///<射击模式
    float Shoot_speed;  ///<射击速度
    uint8_t Shoot_freq; ///<射击频率
	uint8_t Shoot_trig_bit;
	uint8_t FricSwitch;
    ///数据标志
    uint32_t UpdateTime;
    ///日志系统使用
    uint8_t Error_code = 0;          ///<错误代码
    int16_t CAN1_motorlist = 0xffff; ///< CAN1电机列表
    int16_t CAN2_motorlist = 0xffff; ///< CAN2电机列表
	//
	int16_t id_got;
    //本结构体信息
    uint8_t Ready_flag; //就绪标志。表示有新数据未处理。
} sentry_vision_data;

///错误代码列表
enum __app_vision_RobotError
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


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>重要！！！！！需要用户自己实现的函数<<<<<<<<<<<<<<<<<<<<<*/
__weak HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d);

extern uint8_t Vision_IsTxUseDma,Vision_IsRxUseDma;
extern sentry_vision_data VisionTx, VisionRx; ///串口发送/接收缓存结构体
extern uint8_t Vision_Txbuffer[18];           ///串口发送暂存数组
extern uint8_t Vision_RxXfer[APP_VISION_BUFFER_SIZE]; // 串口接收暂存数组
extern int not_analysed_index;  /// Vision_Rxbuffer中从[0]到[not_analysed_index -1]的内容都未曾解析。

extern int not_analysed_index;                  /// Vision_Rxbuffer中从[0]到[not_analysed_index -1]的内容都未曾解析。
#if(APP_VISION_USE_SEMAPHORE)
	extern SemaphoreHandle_t app_vision_uart_semaphore;
#endif

void app_vision_Init(void); ///视觉串口初始化
void app_vision_It(void);   ///视觉串口中断处理

void app_vision_another_Init(void);
void app_vision_dma_rx_cpltcallback(UART_HandleTypeDef *huart); ///
#if(APP_VISION_USE_SEMAPHORE)
void app_vision_dma_tx_cpltcallback(UART_HandleTypeDef *huart);
#endif
void app_vision_dma_rx_abort_in_idle(void);
uint8_t app_vision_analysis_intgrated(void);
//void CMD_SHOOT_ExecuteCallback(float bullet_speed, uint32_t fire_cnt, ShootModeEnum shoot_mode,int16_t ext_trig);

// HAL_StatusTypeDef app_vision_SendData(uint8_t _Functionword);

///视觉传口发送函数
HAL_StatusTypeDef app_vision_SendTxbuffer(uint8_t _Functionword);
void app_vision_load_to_txbuffer(
    uint8_t u8data,
    int loaction_at_buffdata); ///将数据装入缓存Vision_Txbuffer中

void app_vision_load_to_txbuffer(
    float fdata, int loaction_at_buffdata); ///将数据装入缓存Vision_Txbuffer中


void app_vision_test(int id, int cnt);
void APP_TEST_Rx(uint8_t *RxXer);
void APP_TEST_Tx(int id,int size);



















// 各个功能字单独的发送函数
void CMD_GET_MCU_STATE_Tx(float pitch, float, float yaw_soft,
                          uint8_t cloud_mode, uint8_t shoot_mode);
void ROBOT_ERR_Tx(uint8_t err_code);
void STA_CHASSIS_Tx(uint8_t chassis_mode, uint8_t pillar_flag, float velocity,
                    float position);

void JUD_GAME_STATUS_Tx(uint8_t game_progress, uint16_t stage_remain_time);
void JUD_ENY_HP_Tx(uint16_t hp);
void JUD_GAME_EVENT_Tx(); // 待定
void JUD_SELF_HP_Tx(uint16_t hp);
void JUD_GUN_CHASSIS_HEAT_Tx(float chassis_power, uint16_t cha_pwr_buf,
                             uint16_t gun_heat);
void JUD_SELF_BUFF_Tx(uint8_t buff_code);
void JUD_TAKING_DMG_Tx(uint8_t armor_id_enum, uint8_t hurt_type_enum);
void JUD_SHOOTING_Tx(uint8_t bullet_freq, float bullet_speed);
void JUD_AMMO_LEFT_Tx(uint16_t bulelt_left);
void JUD_USER_Tx();

void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer);


uint8_t app_vision_Analysis(void);
extern int not_analysed_index;
#endif
