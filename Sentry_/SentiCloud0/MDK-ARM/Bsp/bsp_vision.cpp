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
#include "bsp_vision.hpp"
//#define DEBUG
#ifdef DEBUG
		uint8_t m = 1;
		float f1 = 12.5f , f2 = 25.2;
#endif

bsp_vision_data bsp_vision_Rec_Data, bsp_vision_Send_Data;    //视觉串口解析到的数据,视觉串口发送的数据
uint8_t Vision_Rxbuffer[BSP_VISION_BUFFER_SIZE] = {0}; //串口接收数据缓存数组，现在缓冲区可以连续接收三帧的数据
static int8_t Array_index = 0;                                //缓冲区数据检测用指针
/**
  * @brief 数据帧类成员函数
  */
//void VisionFrame::pack()
//{
//    int sum = 0, i = 0;
//    for (i = 0; i < Sum_check; i++)
//    {
//        sum += Frame[i];
//    }
//    sum += FRAME_END_DATA;
//    Frame[Sum_check] = sum;
//}
//int8_t VisionFrame::load(uint8_t *ptrData, size_t size)
//{
//    if ((load_iter + size) < Sum_check)
//    {
//        memcpy(ptrData + load_iter, ptrData, size);
//        return 0;
//    }
//    else
//    {
//        return -1;
//    }
//}
/**
  * @brief  云台相对角度控制
  * @details  
  */
//void CMD_GIMBAL_RELATIVE_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Pitch, Vision_Rxbuffer + Array_index + 2, 4); //云台角度解析
//    memcpy(&bsp_vision_Rec_Data.Yaw, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&bsp_vision_Rec_Data.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1); //云台模式解析
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1); //射击模式
//    bsp_vision_Rec_Data.Ready_flag = 1;                                             //数据就绪
//}
///**
//  * @brief  云台绝对角度控制
//  */
//void CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Yaw, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&bsp_vision_Rec_Data.Pitch, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&bsp_vision_Rec_Data.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1);
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1);
//    bsp_vision_Rec_Data.Ready_flag = 1; //数据就绪
//}
///**
//  * @brief  射击控制
//  */
//void CMD_SHOOT_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_SHOOT;
//    bsp_vision_Rec_Data.Shoot_flag = 1;                                             //射击指令就绪
//    memcpy(&bsp_vision_Rec_Data.Shoot_speed, Vision_Rxbuffer + Array_index + 2, 4); //射击速度
//    memcpy(&bsp_vision_Rec_Data.Shoot_freq, Vision_Rxbuffer + Array_index + 6, 1);  //射击频率
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 7, 1);  //射击模式
//}
///**
//  * @brief  底盘运动控制
//  */
//void CMD_CHASSIS_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Vx, Vision_Rxbuffer + Array_index + 2, 4); //底盘速度解析
//    memcpy(&bsp_vision_Rec_Data.Vy, Vision_Rxbuffer + Array_index + 6, 4);
//}
///**
//  * @brief  底盘路程控制
//  */
//void CMD_CHASSIS_LOACTION_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_LOACTION_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Px, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&bsp_vision_Rec_Data.Py, Vision_Rxbuffer + Array_index + 6, 4);
//}
//void CMD_CHASSIS_LOCATION_LIMIT_SPEED_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
//    memcpy(&bsp_vision_Rec_Data.Px, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&bsp_vision_Rec_Data.SpeedLimit, Vision_Rxbuffer + Array_index + 6, 4);
//}
/**
* @brief  视觉串口解析函数
* @details  对缓冲池中数据进行一次遍历并解析数据
* @param  NULL
* @retval  uint8_t 1还没有解析完整个缓冲池数据 0解析已经完成
*/
static uint8_t bsp_vision_Analysis(void)
{
    int16_t _check_sum = 0;                         //和校验用变量
    if (Array_index <= BSP_VISION_BUFFER_SIZE - 18) //缓冲区内开始检测
    {
        //缓冲区里检测数据帧，首先要帧头帧尾对了才开始解包
        if ((Vision_Rxbuffer[Array_index + Frame_header] == FRAME_HEADER_DATA) && (Vision_Rxbuffer[Array_index + Frame_end] == FRAME_END_DATA))
        {
            //帧头帧尾对了，检测一次和校验
            for (int i = 0; i < 18; i++)
            {
                if (i != Sum_check)
                    _check_sum += Vision_Rxbuffer[Array_index + i];
            }
            _check_sum = _check_sum & 0xff;
            if (_check_sum != Vision_Rxbuffer[Array_index + Sum_check])
            {
                Array_index += 1; //和校验错了，指针移动1位继续重新检测
                return 1;         //校验和出错，直接退出,继续在缓冲区检测数据帧
            }

            //帧头帧尾正确，和校验正确，开始解析
            bsp_vision_Rec_Data.Ready_flag = 1; //标记数据就绪
//            switch (Vision_Rxbuffer[Array_index + Function_word])
//            {
//            case CMD_GIMBAL_RELATIVE_CONTROL: //控制云台相对角度，数据解析
//                bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//                CMD_GIMBAL_RELATIVE_CONTROL_Analysis();
//                break;
//            case CMD_GIMBAL_ABSOLUTE_CONTROL: //控制云台绝对角度，数据解析
//                bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//                CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis();
//                break;
//            case CMD_SHOOT: //射击指令
//                bsp_vision_Rec_Data.Function_word = CMD_SHOOT;
//                CMD_SHOOT_Analysis();
//                break;
//            case CMD_CHASSIS_CONTROL: //底盘控制
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//                CMD_CHASSIS_CONTROL_Analysis();
//                break;
//            case CMD_CHASSIS_LOACTION_CONTROL: //底盘路程控制
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//                CMD_CHASSIS_LOACTION_CONTROL_Analysis();
//                break;
//            case CMD_CHASSIS_LOCATION_LIMIT_SPEED: //底盘控制路程带限速
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
//                CMD_CHASSIS_LOCATION_LIMIT_SPEED_Analysis();
//                break;
//            default:
//                bsp_vision_Rec_Data.Ready_flag = 0; //没解析到，取消标记数据就绪
//                break;
//            }
			SentryVisionUartRxAll(Vision_Rxbuffer+Array_index);
            //运行到这里就表示解析已经成功，一帧数据已经完备
            Array_index += 18; //数据帧一帧长度为18，所以移动18位
            return 1;          //处理完一帧，移动18位继续检测
        }
        else
        {
            Array_index += 1; //帧头帧尾不对，移动一位继续检测
            return 1;
        }
    }
    else //余下的数据明显不能构成一帧了，结束解析
    {
        Array_index = 0; //将指针复位
        return 0;        //已经遍历一遍缓冲区，结束处理
    }
}

/**
* @brief  视觉串口初始化
* @details  对视觉和32通讯的串口进行初始化
* @param  NULL
* @retval  NULL
*/
void bsp_vision_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //清除空闲中断位
    __HAL_UART_ENABLE_IT(&BSP_VISION_UART, UART_IT_IDLE);                                       //使能DMA接收空闲中断
    HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //开始DMA接收
}

/**
* @brief  视觉串口中断处理函数
* @details  用于对视觉发送过来的数据做解析操作，将会遍历一次缓冲池
* @param  NULL
* @retval  NULL
*/
void bsp_vision_It(void)
{
    if (__HAL_UART_GET_FLAG(&BSP_VISION_UART, UART_FLAG_IDLE) != RESET) //如果产生了空闲中断
    {
        HAL_UART_DMAStop(&BSP_VISION_UART); //关闭DMA
        while (bsp_vision_Analysis())
            ;                                                                                       //数据解析，遍历一次缓冲区
        memset(Vision_Rxbuffer, 0, BSP_VISION_BUFFER_SIZE);                                         //解析完成，数据清0
        __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //清除空闲中断标志位
        HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //重新开启DMA接收传输
    }
}

/**
* @brief  视觉串口发送函数
* @details  处理给视觉发送的心跳包和日志系统，需要使用者对结构体自行填充数据
* @param  uint8_t _Functionword 功能字列表
* @retval  HAL_StatusTypeDef HAL_OK 发送成功 HAL_ERROR发送失败
*/
uint8_t Vision_Txbuffer[18] = {0}; //发送用数组
HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword)
{
    int16_t _check_sum = 0;         //和校验用变量
    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    Vision_Txbuffer[Frame_header] = FRAME_HEADER_DATA;
    Vision_Txbuffer[Frame_end] = FRAME_END_DATA;
    Vision_Txbuffer[Function_word] = _Functionword;

    switch (_Functionword)
    {
    case CMD_GET_MCU_STATE:         
	//给视觉发心跳包
#ifndef DEBUG
		memcpy(Vision_Txbuffer + 2, &bsp_vision_Send_Data.Cloud_mode, 1);   //控制模式
        memcpy(Vision_Txbuffer + 3, &bsp_vision_Send_Data.Pitch, 4);        //Pitch轴数据
        memcpy(Vision_Txbuffer + 7, &bsp_vision_Send_Data.Yaw, 4);          //Yaw轴数据
#else
		memcpy(Vision_Txbuffer + 2, &m, 1);   //控制模式
        memcpy(Vision_Txbuffer + 3, &f1, 4);        //Pitch轴数据
        memcpy(Vision_Txbuffer + 7, &f2, 4);          //Yaw轴数据
#endif
        memcpy(Vision_Txbuffer + 11, &bsp_vision_Send_Data.Shoot_speed, 4); //射速
        memcpy(Vision_Txbuffer + 15, &bsp_vision_Send_Data.Shoot_freq, 1);  //射频
        break;
    case ROBOT_ERR:
        memcpy(Vision_Txbuffer + 2, &bsp_vision_Send_Data.Error_code, 1); //日志系统错误代码
        if (bsp_vision_Send_Data.Error_code == MOTOR_OFFLINE_CNT)         //0x04的时候要额外发两个在线列表数据
        {
            memcpy(Vision_Txbuffer + 3, &bsp_vision_Send_Data.CAN1_motorlist, 2); //CAN1电机数据
            memcpy(Vision_Txbuffer + 5, &bsp_vision_Send_Data.CAN2_motorlist, 2); //CAN2电机数据
        }
        break;
    case STA_CHASSIS:
        Vision_Txbuffer[2] = 0;
        memcpy(Vision_Txbuffer + 3, &bsp_vision_Send_Data.pillar_flag, 1);
        memcpy(Vision_Txbuffer + 4, &bsp_vision_Send_Data.Px, 4);
    default:
        break;
    }
    for (int i = 0; i < 18; i++)
    {
        if (i != Sum_check)
            _check_sum += Vision_Txbuffer[i];
    }
    _check_sum = _check_sum & 0xff;
    Vision_Txbuffer[Sum_check] = _check_sum;

    return HAL_UART_Transmit_DMA(&BSP_VISION_UART, Vision_Txbuffer, 18);
}

