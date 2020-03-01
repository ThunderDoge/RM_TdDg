/** 
 * @file: app_vision.cpp 
 * @encoding
* @brief    视觉串口协议文件
* @details  32与小主机的通讯处理
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020电控
* @par 日志
    2019/12/8   将Evan-GH个人串口版本1.2合并到公共代码库串口v2.1.0
    2019/12/8   ThunderDoge依据自己的喜好将串口库更改为v2.1.1   增加了部分功能。修改了一些写法

    PS 与视觉的通信协议参见《RM2020基本视觉协议 v2.0》 by Evan-GH
*/

///依赖的文件
#include "app_vision.hpp"
//#include "SentryCommu.hpp"

//调试用标志位
#define VIUART_DISABLE_OTHER_IT

sentry_vision_data VisionRx, VisionTx; ///视觉串口解析到的数据,视觉串口发送的数据
uint8_t Vision_Rxbuffer[BSP_VISION_BUFFER_SIZE] = {0};     ///串口接收数据缓存数组，现在缓冲区可以连续接收三帧的数据
static int8_t Array_index = 0;                             ///缓冲区数据检测用指针

void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer);   ///统一的接收缓存处理函数。！Vision_Rxbuffer应当验证通过已符合数据帧帧格式

/**
* @brief  视觉串口解析函数
* @details  在中断中，对缓冲池中数据进行一次遍历并解析数据
* @param  NULL
* @retval  uint8_t 1还没有解析完整个缓冲池数据 0解析已经完成
*/
static uint8_t bsp_vision_Analysis(void)
{
    int16_t _check_sum = 0;                         //和校验用变量
    if (Array_index <= BSP_VISION_BUFFER_SIZE - 18) //检查未超过下标
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
            if (_check_sum != Vision_Rxbuffer[Array_index + Sum_check]) //检查和校验
            {
                Array_index += 1; //和校验错了，指针移动1位继续重新检测
                return 1;         //校验和出错，直接退出,继续在缓冲区检测数据帧
            }

            //帧头帧尾正确，和校验正确，开始解析
            VisionRx.Ready_flag = 1; //标记数据就绪
            SentryVisionUartRxAll(Vision_Rxbuffer + Array_index);   //解析数据帧
            //运行到这里就表示解析已经成功，一帧数据已经完备
            Array_index += 18; //数据帧一帧长度为18，所以移动18位
            return 1;          //处理完一帧，移动18位继续检测
        }
        else
        {
            Array_index += 1; //帧头帧尾不对，移动一位继续检测
            return 1;   //返回1表示解析未完成，要求继续循环解析
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
* @details  重置空闲(IDLE)中断位，开启空闲中断。关闭其他的无关中断。开启UART-DMA接收，绑定到Vision_Rxbuffer。长度设置为BSP_VISION_BUFFER_SIZE。
* @param  NULL
* @retval  NULL
*/
void bsp_vision_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);          //清除空闲中断位
    __HAL_UART_ENABLE_IT(&BSP_VISION_UART, UART_IT_IDLE); //使能DMA接收空闲中断

#ifdef VIUART_DISABLE_OTHER_IT //除能其他所有的中断
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_CTS);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_LBD);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_TC);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_ERR);
#endif

    HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //开始DMA接收，DMA连接到Vision_Rxbuffer
}

/**
* @brief  视觉串口中断处理函数
* @details  用于对视觉发送过来的数据做解析操作，将会遍历一次缓冲池
* @param  NULL
* @retval  NULL
*/
void bsp_vision_It(void)
{
#ifndef VIUART_DISABLE_OTHER_IT                                         //除能其他所有的中断，不再需要判断
    if (__HAL_UART_GET_FLAG(&BSP_VISION_UART, UART_FLAG_IDLE) != RESET) //如果产生了空闲中断
#endif
    {
        HAL_UART_DMAStop(&BSP_VISION_UART); //关闭DMA
        while (bsp_vision_Analysis())
            ;                                                                                       //数据解析，遍历一次缓冲区
        memset(Vision_Rxbuffer, 0, BSP_VISION_BUFFER_SIZE);                                         //解析完成，数据清0
        __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //清除空闲中断标志位
        HAL_UART_DMAResume(&BSP_VISION_UART);                                                       //重新打开DMA
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
/**
 * @brief 将数据装入缓存Vision_Txbuffer中
 * 
 * @param     u8data 这一字节的数据
 * @param     location_at_buffdata 在数据帧【数据】段中的位置。0表示【数据】第1字节. 类推
 */
void bsp_vision_load_to_txbuffer(uint8_t u8data, int location_at_buffdata)
{
    Vision_Txbuffer[location_at_buffdata + 2] = u8data;
}
/**
 * @brief 
 * 
 * @param     fdata 待装入的浮点数据
 * @param     location_at_buffdata 在数据帧【数据】段中的位置。0表示装入【数据】第1~4字节. 类推. 注意1个float占用4字节
 */
void bsp_vision_load_to_txbuffer(float fdata, int location_at_buffdata)
{
    //    *((float*)(&Vision_Txbuffer[location_at_buffdata+2])) = fdata;	This cause HardFault
    memcpy(Vision_Txbuffer + location_at_buffdata + 2, &fdata, 4);
}
/**
 * @brief 发送数据帧
 * 
 * @param     _Functionword 待发送数据帧的功能字。
 * @return HAL_StatusTypeDef HAL_OK为正常
 */
HAL_StatusTypeDef bsp_vision_SendTxbuffer(uint8_t _Functionword)
{
    int16_t _check_sum = 0; //和校验用变量
    // memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    Vision_Txbuffer[Frame_header] = FRAME_HEADER_DATA;
    Vision_Txbuffer[Frame_end] = FRAME_END_DATA;
    Vision_Txbuffer[Function_word] = _Functionword;
    for (int i = 0; i < 18; i++)
    {
        if (i != Sum_check)
            _check_sum += Vision_Txbuffer[i];
    }
    _check_sum = _check_sum & 0xff;
    Vision_Txbuffer[Sum_check] = _check_sum;
    //Check if UART Ready;
    while (HAL_UART_GetState(&BSP_VISION_UART) == HAL_UART_STATE_BUSY_TX ||
           HAL_UART_GetState(&BSP_VISION_UART) == HAL_UART_STATE_BUSY_TX_RX)
    {
        // s1 = HAL_UART_GetState(&BSP_VISION_UART);
#ifdef INC_FREERTOS_H
        vTaskDelay(1);
//#else
//	HAL_Delay(1);
#endif
    }

    return HAL_UART_Transmit_DMA(&BSP_VISION_UART, Vision_Txbuffer, 18);
}
#define USE_VISION
#ifdef USE_VISION
//VisionUart TxRx Functions 视觉串口函数
//VisionUart Recv 视觉串口接收函数
///云台相对角度控制
void CMD_GIMBAL_RELATIVE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_RELATIVE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 2, 4); //云台角度解析
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1); //云台模式解析
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1); //射击模式
        VisionRx.cloud_ctrl_mode = relative_cloud;                  //数据就绪
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///云台绝对角度控制
void CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_ABSOLUTE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1);
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1);
        VisionRx.cloud_ctrl_mode = absolute_cloud; //数据就绪
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///云台速度控制
void CMD_GIMBAL_SPEED_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
	if(Vision_Rxbuffer[Function_word] == CMD_GIMBAL_SPEED_CONTROL)
	{
		VisionRx.Function_word = CMD_GIMBAL_SPEED_CONTROL;
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1);
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1);
		VisionRx.cloud_ctrl_mode = speed_cloud;
        VisionRx.UpdateTime = HAL_GetTick();
	}
}
///射击控制
void CMD_SHOOT_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_SHOOT)
    {
        VisionRx.Function_word = CMD_SHOOT;
        VisionRx.Shoot_mode = 1;                               //射击指令就绪
        memcpy(&VisionRx.Shoot_speed, Vision_Rxbuffer + 2, 4); //射击速度
        memcpy(&VisionRx.Shoot_freq, Vision_Rxbuffer + 6, 1);  //射击频率
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 7, 1);  //射击模式
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///底盘运动控制
void CMD_CHASSIS_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_CONTROL)
    {
        VisionRx.Function_word = CMD_CHASSIS_CONTROL;
        memcpy(&VisionRx.Vx, Vision_Rxbuffer + 2, 4); //底盘速度解析
        memcpy(&VisionRx.Vy, Vision_Rxbuffer + 6, 4);
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///底盘路程控制
void CMD_CHASSIS_LOACTION_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_LOACTION_CONTROL)
    {
        VisionRx.Function_word = CMD_CHASSIS_LOACTION_CONTROL;
        memcpy(&VisionRx.Px, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Py, Vision_Rxbuffer + 6, 4);
        VisionRx.chassis_mode = _chassis_location;
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///底盘路程控制且限速
void CMD_CHASSIS_LOCATION_LIMIT_SPEED_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_LOCATION_LIMIT_SPEED)
    {
        VisionRx.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
        memcpy(&VisionRx.Px, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.SpeedLimit, Vision_Rxbuffer + 6, 4);
        VisionRx.chassis_mode = _chassis_location_limit_speed;
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///全命令接收	新增的功能字接收函数请在这里面调用
///视觉串口中断接收函数
void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer)
{
    CMD_GIMBAL_RELATIVE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_SHOOT_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOACTION_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOCATION_LIMIT_SPEED_Rx(Vision_Rxbuffer);
	CMD_GIMBAL_SPEED_CONTROL_Rx(Vision_Rxbuffer);
}
///视觉串口发送函数
void CMD_GET_MCU_STATE_Tx()
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer((uint8_t)VisionTx.Cloud_mode, 0U);
    bsp_vision_load_to_txbuffer(VisionTx.Pitch, 1);
    bsp_vision_load_to_txbuffer(VisionTx.Yaw, 5);
    bsp_vision_load_to_txbuffer(VisionTx.YawSoft, 9U);
    bsp_vision_load_to_txbuffer(VisionTx.Shoot_mode, 13U);
    bsp_vision_SendTxbuffer(CMD_GET_MCU_STATE);
}
///串口发送到小主机日志系统
void ROBOT_ERR_Tx()
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer(VisionTx.Error_code, 0U);
    bsp_vision_SendTxbuffer(ROBOT_ERR);
}
///发送底盘状态
void STA_CHASSIS_Tx()
{
		while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    bsp_vision_load_to_txbuffer(VisionTx.chassis_mode, 0U);
    bsp_vision_load_to_txbuffer(VisionTx.pillar_flag, 1U);
    bsp_vision_load_to_txbuffer(VisionTx.Vx, 2);
	bsp_vision_load_to_txbuffer(VisionTx.Px, 6);
    bsp_vision_SendTxbuffer(STA_CHASSIS);
}
///回调函数，直接执行接收到的小主机指令
void VisionRxHandle(void)
{
    //云台指令处理
    switch (VisionRx.cloud_ctrl_mode)
    {
        case relative_cloud:
            CloudEntity.SetAngleTo(CloudEntity.RealPitch + VisionRx.Pitch,
                            CloudEntity.RealYaw + VisionRx.Yaw);
            break;
        case absolute_cloud:
            CloudEntity.SetAngleTo(VisionRx.Pitch, VisionRx.Yaw);
			break;
		case speed_cloud:
			CloudEntity.PitchMotor.Angle_Set(CloudEntity.RealPitch + VisionRx.Pitch);
			CloudEntity.YawMotor.Speed_Set(VisionRx.Yaw);
			break;
        default:
            break;
    }
    VisionRx.cloud_ctrl_mode = 0;    //处理完成标志。因为一个命令只会处理一次，处理后置0
}
///与小主机通信任务用的回调函数
void CloudVisonTxRoutine(void)
{
    //装载各种信息
    VisionTx.Cloud_mode = CloudEntity.Mode;
    VisionTx.Shoot_mode = CloudEntity.shoot_flag;
    VisionTx.Pitch = CloudEntity.RealPitch;
    VisionTx.YawSoft = CloudEntity.RealYaw;
	VisionTx.Yaw = CloudEntity.MechanicYaw;
    VisionTx.Shoot_speed = 0;

    VisionTx.Error_code = 0;

    VisionTx.chassis_mode = VisionRx.chassis_mode;
    VisionTx.Vx = CanRx.Chassis_SpeedLocation[0];
    VisionTx.pillar_flag = CanRx.Pillar_flag;
    VisionTx.Px = CanRx.Chassis_SpeedLocation[1];
	VisionTx.pillar_flag = CanRx.Pillar_flag;

    //串口发送
    CMD_GET_MCU_STATE_Tx();
    ROBOT_ERR_Tx();
    STA_CHASSIS_Tx();
}

#endif //USE_VISION


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
//    VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//    memcpy(&VisionRx.Pitch, Vision_Rxbuffer + Array_index + 2, 4); //云台角度解析
//    memcpy(&VisionRx.Yaw, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1); //云台模式解析
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1); //射击模式
//    VisionRx.Ready_flag = 1;                                             //数据就绪
//}
///**
//  * @brief  云台绝对角度控制
//  */
//void CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//    memcpy(&VisionRx.Yaw, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&VisionRx.Pitch, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1);
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1);
//    VisionRx.Ready_flag = 1; //数据就绪
//}
///**
//  * @brief  射击控制
//  */
//void CMD_SHOOT_Analysis()
//{
//    VisionRx.Function_word = CMD_SHOOT;
//    VisionRx.Shoot_flag = 1;                                             //射击指令就绪
//    memcpy(&VisionRx.Shoot_speed, Vision_Rxbuffer + Array_index + 2, 4); //射击速度
//    memcpy(&VisionRx.Shoot_freq, Vision_Rxbuffer + Array_index + 6, 1);  //射击频率
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 7, 1);  //射击模式
//}
///**
//  * @brief  底盘运动控制
//  */
//void CMD_CHASSIS_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_CHASSIS_CONTROL;
//    memcpy(&VisionRx.Vx, Vision_Rxbuffer + Array_index + 2, 4); //底盘速度解析
//    memcpy(&VisionRx.Vy, Vision_Rxbuffer + Array_index + 6, 4);
//}
///**
//  * @brief  底盘路程控制
//  */
//void CMD_CHASSIS_LOACTION_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_CHASSIS_LOACTION_CONTROL;
//    memcpy(&VisionRx.Px, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&VisionRx.Py, Vision_Rxbuffer + Array_index + 6, 4);
//}
//void CMD_CHASSIS_LOCATION_LIMIT_SPEED_Analysis()
//{
//    VisionRx.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
//    memcpy(&VisionRx.Px, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&VisionRx.SpeedLimit, Vision_Rxbuffer + Array_index + 6, 4);
//}
//HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword)
//{
//    int16_t _check_sum = 0;         //和校验用变量
//    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
//    Vision_Txbuffer[Frame_header] = FRAME_HEADER_DATA;
//    Vision_Txbuffer[Frame_end] = FRAME_END_DATA;
//    Vision_Txbuffer[Function_word] = _Functionword;

//    switch (_Functionword)
//    {
//    case CMD_GET_MCU_STATE:
//        //给视觉发心跳包
//#ifndef DEBUG
//        memcpy(Vision_Txbuffer + 2, &VisionTx.Cloud_mode, 1); //控制模式
//        memcpy(Vision_Txbuffer + 3, &VisionTx.Pitch, 4);      //Pitch轴数据
//        memcpy(Vision_Txbuffer + 7, &VisionTx.Yaw, 4);        //Yaw轴数据
//#else
//        memcpy(Vision_Txbuffer + 2, &m, 1);  //控制模式
//        memcpy(Vision_Txbuffer + 3, &f1, 4); //Pitch轴数据
//        memcpy(Vision_Txbuffer + 7, &f2, 4); //Yaw轴数据
//#endif
//        memcpy(Vision_Txbuffer + 11, &VisionTx.Shoot_speed, 4); //射速
//        memcpy(Vision_Txbuffer + 15, &VisionTx.Shoot_freq, 1);  //射频
//        break;
//    case ROBOT_ERR:
//        memcpy(Vision_Txbuffer + 2, &VisionTx.Error_code, 1); //日志系统错误代码
//        if (VisionTx.Error_code == MOTOR_OFFLINE_CNT)         //0x04的时候要额外发两个在线列表数据
//        {
//            memcpy(Vision_Txbuffer + 3, &VisionTx.CAN1_motorlist, 2); //CAN1电机数据
//            memcpy(Vision_Txbuffer + 5, &VisionTx.CAN2_motorlist, 2); //CAN2电机数据
//        }
//        break;
//    case STA_CHASSIS:
//        Vision_Txbuffer[2] = 0;
//        memcpy(Vision_Txbuffer + 3, &VisionTx.pillar_flag, 1);
//        memcpy(Vision_Txbuffer + 4, &VisionTx.Px, 4);
//    default:
//        break;
//    }
//    for (int i = 0; i < 18; i++)
//    {
//        if (i != Sum_check)
//            _check_sum += Vision_Txbuffer[i];
//    }
//    _check_sum = _check_sum & 0xff;
//    Vision_Txbuffer[Sum_check] = _check_sum;

//    return HAL_UART_Transmit_DMA(&BSP_VISION_UART, Vision_Txbuffer, 18);
//}
