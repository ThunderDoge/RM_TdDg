/** 
 * @file: app_vision.cpp 
 * @encoding
* @brief    视觉串口协议文件
* @details  32与小主机的通讯处理
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v3.0
* @par Copyright (c):  RM2020电控
* @par 日志
    2019/12/8   将Evan-GH个人串口版本1.2合并到公共代码库串口v2.1.0
    2019/12/8   ThunderDoge依据自己的喜好将串口库更改为v2.1.1   增加了部分功能。修改了一些写法
    v3.0    2020-4-16   重写视觉串口。使用RTOS任务来统一管理串口的发送和接收。
    PS 与视觉的通信协议参见《RM2020基本视觉协议 v2.0》 by Evan-GH
*/



///依赖的文件
#include "app_vision.hpp"
//#include "SentryCommu.hpp"



//调试用标志位
#define VIUART_DISABLE_OTHER_IT
#define _VISION_DEBUG



///储存数据使用的结构体
sentry_vision_data VisionRx, VisionTx; ///视觉串口解析到的数据,视觉串口发送的数据
uint8_t Vision_RxXfer[APP_VISION_BUFFER_SIZE] = {0};     ///串口接收数据缓存数组，现在缓冲区可以连续接收三帧的数据

/// "使用DMA发送" FLAG
uint8_t Vision_IsTxUseDma;
uint8_t Vision_IsRxUseDma;

///关键状态变量
static int8_t Array_index = 0;                             ///缓冲区数据检测用指针
int not_analysed_index=0;                  /// Vision_Rxbuffer中从[0]到[not_analysed_index -1]的内容都未曾解析。

#if(APP_VISION_USE_SEMAPHORE)
SemaphoreHandle_t app_vision_uart_semaphore;
#endif

/// 根据协议解析的函数的集合。在缓存数据解析中调用。
void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer);   ///统一的接收缓存处理函数。！Vision_Rxbuffer应当验证通过已符合数据帧帧格式

/**
* @brief  视觉串口解析函数
*   解包将收到的原始数据转换成可使用的数据
*   关键变量
*   unsigned char _recv_buf[_len*3];  //接收时用于存储原始数据
*   int _recv_len;                //接收缓冲区当前有效字节数量
*   存储结构决定,越早收到的字节在越前面
*   缓冲区维护逻辑
*   1.收到正确帧,正确帧和正确帧之前的内容清空,后面的数据依次向前移动
*   2.缓冲区超过长度,清空最前面的_len个字节,后续字节依次向前移动_len个字节,新收到的内容追加在后部
*   3.帧头帧尾符合,校验和不符合,继续向下检验
*   4.在_recv_buf中只有前_recv_len字节有效,剩余的缓冲区内容无效,不进行维护,禁止读取
* @details  在中断中，对缓冲池中数据进行一次遍历并解析数据
* @param  NULL
* @retval  uint8_t 1还没有解析完整个缓冲池数据 0解析已经完成
*/
uint8_t app_vision_Analysis(void)
{
    int16_t _check_sum = 0;                         //和校验用变量
    if (Array_index <= APP_VISION_BUFFER_SIZE - 18) //检查未超过下标
    {
        //缓冲区里检测数据帧，首先要帧头帧尾对了才开始解包
        if ((Vision_RxXfer[Array_index + Frame_header] == FRAME_HEADER_DATA) && (Vision_RxXfer[Array_index + Frame_end] == FRAME_END_DATA))
        {
            //帧头帧尾对了，检测一次和校验
            for (int i = 0; i < 18; i++)
            {
                if (i != Sum_check)
                    _check_sum += Vision_RxXfer[Array_index + i];
            }
            _check_sum = _check_sum & 0xff;
            if (_check_sum != Vision_RxXfer[Array_index + Sum_check]) //检查和校验
            {
                Array_index += 1; //和校验错了，指针移动1位继续重新检测
                return 1;         //校验和出错，直接退出,继续在缓冲区检测数据帧
            }
			
			SentryVisionUartRxAll(Vision_RxXfer + Array_index);   //解析数据帧
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
* @brief  视觉串口解析函数
*   解包将收到的原始数据转换成可使用的数据
*   关键变量
*   unsigned char _recv_buf[_len*3];  //接收时用于存储原始数据
*   int _recv_len;                //接收缓冲区当前有效字节数量
*   存储结构决定,越早收到的字节在越前面
*   缓冲区维护逻辑
*   1.收到正确帧,正确帧和正确帧之前的内容清空,后面的数据依次向前移动
*   2.缓冲区超过长度,清空最前面的_len个字节,后续字节依次向前移动_len个字节,新收到的内容追加在后部
*   3.帧头帧尾符合,校验和不符合,继续向下检验
*   4.在_recv_buf中只有前_recv_len字节有效,剩余的缓冲区内容无效,不进行维护,禁止读取
* @details  在中断中，对缓冲池中数据进行一次遍历并解析数据
* @param  NULL
* @retval  uint8_t 1还没有解析完整个缓冲池数据 0解析已经完成
*/
uint8_t app_vision_analysis_intgrated(void)
{
    int16_t _check_sum = 0;                         //和校验用变量
    
    uint32_t head_index=0;            //指示目前正在解析的段的起始。
    uint32_t end_index=Frame_end;            //指示目前正在解析的段的结束。
	
	int first_0xFF=-1;		// -1 事初始值,表示没有

    uint8_t frame_solved=0;

    if( not_analysed_index-1 < end_index )   // 本来收到的数据根本不够解析一个帧
    {   return 0;   }

    // 遍历 Vision_Rxbuffer
    for(;
    end_index < not_analysed_index;    // 检查未超过已收到的数据的下标
    head_index++ , end_index++) // 指示变量后移
    {
		if( Vision_RxXfer [ head_index ] == FRAME_HEADER_DATA )   // 是0XFF
		{
            if(Vision_RxXfer [ end_index ] == FRAME_END_DATA)     // 帧尾是 0X0D
            {

                // 计算校验和
                for (int i = head_index; i <= end_index; i++)   
                {
                    if (i != head_index+Sum_check)
                        _check_sum += Vision_RxXfer[i];
                    _check_sum &= 0xff;
                }


                // 如果校验和符合
                if (_check_sum == Vision_RxXfer[ head_index + Sum_check ] ) // 和校验符合
                {
                    //帧头帧尾正确，和校验正确，开始解析
                    VisionRx.Ready_flag ++; //标记数据就绪
                    frame_solved++;          //解出的帧的计数
                    SentryVisionUartRxAll(Vision_RxXfer + head_index);   //解析数据帧
                    //运行到这里就表示解析已经成功，一帧有效数据已经解出
                    head_index = end_index;
                    end_index += Frame_end;

                    // // 如果之前标记了 first_0xFF，要重置它
                    // first_0xFF = -1;
                }
                else
                {
                    continue;   //跳过到下一个
                }

            }
            else    // 符合帧头但是不符合帧尾
            {
                // if(first_0xFF == -1)    // 记录第一个 0XFF
                // {
                //     first_0xFF = head_index;
                // }
                continue;   //跳过到下一个

            }
		}
        else    // 不是 0XFF
        {
            continue;   //跳过到下一个
        }
    }
    // 遍历到此结束

    // 将未解析的数据移到前面
	
	// 第一个0XFF 可能在 不够构成一个帧的长度里面.遍历，检查一下.
	// if(first_0xFF == -1)
	// {
		for(int i=head_index;i<not_analysed_index;i++)
		{
			if(Vision_RxXfer[i] == 0xff)
			{
				first_0xFF = i;
				break;		// 找到就退了
			}
		}
	// }
	//还是没有
	if(first_0xFF == -1)
	{
		first_0xFF = not_analysed_index;	// 这样说明根本没有0XFF,直接扔掉即可
	}
	
	// 移动数据到缓存头部，但是第一个0XFF之前的不要
    int from_index,to_index;
    for(from_index =first_0xFF,to_index=0;
        from_index < not_analysed_index;
        from_index++,to_index++)
    {
        Vision_RxXfer[to_index] = Vision_RxXfer[from_index];
    }
    not_analysed_index = to_index;

    memset(&Vision_RxXfer[not_analysed_index] , 0 , APP_VISION_BUFFER_SIZE - not_analysed_index);


    // 返回解析出的帧的数量。
    return frame_solved;

}
#if(APP_VISION_USE_SEMAPHORE)
/// DMA发送回调函数。释放信号量
void app_vision_dma_tx_cpltcallback(UART_HandleTypeDef *huart)
{
	xSemaphoreGiveFromISR(app_vision_uart_semaphore,NULL);
}
#endif




/**
* @brief  视觉串口初始化
* @details  重置空闲(IDLE)中断位，开启空闲中断。关闭其他的无关中断。开启UART-DMA接收，绑定到Vision_Rxbuffer。长度设置为APP_VISION_BUFFER_SIZE。
* @param  NULL
* @retval  NULL
*/
void app_vision_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&APP_VISION_UART);          //清除空闲中断位
    __HAL_UART_ENABLE_IT(&APP_VISION_UART, UART_IT_IDLE); //使能DMA接收空闲中断

#ifdef VIUART_DISABLE_OTHER_IT //除能其他所有的中断
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_CTS);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_LBD);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_TC);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&APP_VISION_UART, UART_IT_ERR);
#endif
	if(APP_VISION_UART.hdmarx != NULL)
	{
		HAL_UART_Receive_DMA(&APP_VISION_UART, (uint8_t *)Vision_RxXfer, APP_VISION_BUFFER_SIZE); //开始DMA接收，DMA连接到Vision_Rxbuffer
	}
	
#if(APP_VISION_USE_SEMAPHORE)
	app_vision_uart_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(app_vision_uart_semaphore);
#endif
#if(USE_HAL_UART_REGISTER_CALLBACKS == 1)
	HAL_UART_RegisterCallback(&APP_VISION_UART,HAL_UART_RX_COMPLETE_CB_ID,app_vision_dma_tx_cpltcallback);
    HAL_UART_RegisterCallback(&)
#endif
	if(APP_VISION_UART.hdmatx != NULL && APP_VISION_UART.hdmatx->State == HAL_DMA_STATE_READY)
		Vision_IsTxUseDma=1;
	if(APP_VISION_UART.hdmarx != NULL && APP_VISION_UART.hdmarx->State == HAL_DMA_STATE_READY)
		Vision_IsRxUseDma=1;
}

#ifndef __MAIN_DEBUG
#if(USE_HAL_UART_REGISTER_CALLBACKS != 1)
#if(APP_VISION_USE_SEMAPHORE)
#endif
#endif
#endif // __MAIN_DEBUG







/**
* @brief  视觉串口中断处理函数
* @details  用于对视觉发送过来的数据做解析操作，将会遍历一次缓冲池
* @param  NULL
* @retval  NULL
*/
void app_vision_It(void)
{
//#ifndef VIUART_DISABLE_OTHER_IT                                         //除能其他所有的中断，不再需要判断
//    if (__HAL_UART_GET_FLAG(&APP_VISION_UART, UART_FLAG_IDLE) != RESET) //如果产生了空闲中断
//#endif
//     {
//         HAL_UART_DMAStop(&APP_VISION_UART); //关闭DMA
//         while (app_vision_Analysis())
//             ;                                                                                       //数据解析，遍历一次缓冲区
// //        memset(Vision_RxXfer, 0, APP_VISION_BUFFER_SIZE);                                         //解析完成，数据清0
//         __HAL_UART_CLEAR_IDLEFLAG(&APP_VISION_UART);                                                //清除空闲中断标志位
//         HAL_UART_DMAResume(&APP_VISION_UART);                                                       //重新打开DMA
//         HAL_UART_Receive_DMA(&APP_VISION_UART, (uint8_t *)Vision_RxXfer, APP_VISION_BUFFER_SIZE); //重新开启DMA接收传输
//     }
	if(__HAL_UART_GET_FLAG(&APP_VISION_UART,UART_FLAG_IDLE) != RESET)	//如果产生了空闲中断
	{
		HAL_UART_DMAStop(&APP_VISION_UART); //关闭DMA
		while(app_vision_Analysis());	//数据解析，遍历一次缓冲区
		memset(Vision_RxXfer,0,APP_VISION_BUFFER_SIZE); //解析完成，数据清0
		__HAL_UART_CLEAR_IDLEFLAG(&APP_VISION_UART); //清除空闲中断标志位
		HAL_UART_DMAResume(&APP_VISION_UART);         //重新打开DMA，这句记得加
		HAL_UART_Receive_DMA(&APP_VISION_UART, (uint8_t*)Vision_RxXfer, APP_VISION_BUFFER_SIZE);//重新开启DMA接收传输
	}

}



/**
 * @brief 接收的数量
 * 
 */
void app_vision_dma_rx_abort_in_idle(void)
{
    // 将已收到的数据登记到未解析数据
    not_analysed_index += APP_VISION_UART.RxXferSize -__HAL_DMA_GET_COUNTER(APP_VISION_UART.hdmarx);
    
    HAL_UART_AbortReceive_IT(&APP_VISION_UART); // 停止收，这个函数可以用于DMA接收和中断接收的停止。详见此函数注释。

    app_vision_analysis_intgrated();            //解析. 在里面 not_analysed_index 更新了
    
    HAL_UART_Receive_DMA(&APP_VISION_UART,      //重启接收
        Vision_RxXfer+not_analysed_index,       //not_analysed_indexer 处开始写
        APP_VISION_BUFFER_SIZE-not_analysed_index); // 接收的数量一直到填满 Vision_Rxbuffer
    
    return ;
}


/**
 * @brief 小主机通信 串口中断DMA接收完成 回调函数。
 * 在 HAL_UART_RxCpltCallback 中调用此函数，前面要加上 if huart1。
 * 
 * 正确运行应该接收满整个缓存才会调用此回调函数。
 * 启动数据解析。
 * 
 */
void app_vision_dma_rx_cpltcallback(UART_HandleTypeDef *huart)
{
	not_analysed_index = APP_VISION_BUFFER_SIZE;	// 直接设定整个缓存为为未解析。因为DMA/IT接收启动时 设定是接受满缓存才会接收完成返回。
	
    app_vision_analysis_intgrated();            //解析
    
	if(Vision_IsRxUseDma)
	{
    HAL_UART_Receive_DMA(&APP_VISION_UART,      //重启接收
        Vision_RxXfer+not_analysed_index,       //not_analysed_indexer 处开始写
        APP_VISION_BUFFER_SIZE-not_analysed_index+1); // 接收的数量一直到填满 Vision_Rxbuffer
	}
	else
	{
    HAL_UART_Receive_IT(&APP_VISION_UART,      //重启接收
        Vision_RxXfer+not_analysed_index,       //not_analysed_indexer 处开始写
        APP_VISION_BUFFER_SIZE-not_analysed_index+1); // 接收的数量一直到填满 Vision_Rxbuffer
	}

    return ;
}













//-------------------------------------下面是视觉串口 根据协议发送的函数--------------------------------------------------


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
void app_vision_load_to_txbuffer(uint8_t u8data, int location_at_buffdata)
{
    Vision_Txbuffer[location_at_buffdata + 2] = u8data;
}
/**
 * @brief 
 * 
 * @param     fdata 待装入的浮点数据
 * @param     location_at_buffdata 在数据帧【数据】段中的位置。0表示装入【数据】第1~4字节. 类推. 注意1个float占用4字节
 */
void app_vision_load_to_txbuffer(float fdata, int location_at_buffdata)
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
HAL_StatusTypeDef app_vision_SendTxbuffer(uint8_t _Functionword)
{
	int16_t _check_sum = 0; //和校验用变量

	#if(APP_VISION_USE_SEMAPHORE)
		if(xSemaphoreTake(app_vision_uart_semaphore,10) == errQUEUE_EMPTY)	// 获取信号量
		{
			HAL_UART_AbortTransmit_IT(&APP_VISION_UART);	// 超时未得到信号量，会强行终止串口发送
		}
    #else
//        if(APP_VISION_UART.gState )
	#endif

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

    return HAL_UART_Transmit_DMA(&APP_VISION_UART, Vision_Txbuffer, 18);
}





///云台相对角度控制
void CMD_GIMBAL_RELATIVE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_RELATIVE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 2, 4); //云台角度解析
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Shoot_trig_bit, Vision_Rxbuffer + 10, 1); 
        memcpy(&VisionRx.Shoot_gap, Vision_Rxbuffer + 11, 4); 
        memcpy(&VisionRx.FricSwitch, Vision_Rxbuffer + 15, 1);
        VisionRx.FricSwitch = !VisionRx.FricSwitch; // 摩擦轮开关取反
        VisionRx.cloud_ctrl_mode = relative_cloud;                  //数据就绪
		
//		CMD_SHOOT_ExecuteCallback((VisionRx.FricSwitch>0)*4000,1U,ShtOnce,(VisionRx.Shoot_trig_bit>0)*400);
		CMD_SHOOT_ExecuteCallback((VisionRx.FricSwitch>0)*4000,10,VisionRx.Shoot_gap,ShtBurst,(VisionRx.Shoot_trig_bit>0)*400);
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
/**
 * @brief       触发射击，执行回调函数
 * 以下的参数的具体含义需要在定义处写明
 * 您不应该修改此定义。您应该在另处实现此函数的功能，因为此处的定义是WEAK的
 * @param     bullet_speed  子弹飞行速度
 * @param     fire_freq     子弹射击频率
 * @param     shoot_mode    子弹射击模式。
 */
__weak void CMD_SHOOT_ExecuteCallback(float bullet_speed, uint32_t fire_cnt,uint32_t shoot_gap, ShootModeEnum shoot_mode,int16_t ext_trig)
{
	// 您不应该修改此函数。您应该在另处实现此函数的功能，因为此处的定义是WEAK的
}

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
//		CMD_SHOOT_ExecuteCallback(VisionRx.Shoot_speed,VisionRx.Shoot_freq,VisionRx.Shoot_mode);
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



/**
 * @brief       读取PID参数的回调函数。由外面实现
 * 
 * @param     pid_id    PID编号
 * @param     p         读取后写入P的地址
 * @param     i         读取后写入I的地址
 * @param     d         读取后写入D的地址
 * @return 状态值。正常返回HAL_OK, 异常返回HAL_ERROR 
 */
__weak HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d)
{
    //在这里添加你自己的实现。
    UNUSED(pid_id);
    UNUSED(p);
    UNUSED(i);
    UNUSED(d);
    return HAL_ERROR;
}



/**
 * @brief       发送 CMD_READ_PID 命令返回值的函数
 * 
 * @param     pid_id    PID编号
 * @param     p         读取到的P
 * @param     i         读取到的I
 * @param     d         读取到的D
 */
void CMD_READ_PID_Tx(uint8_t pid_id,float p,float i,float d)
{
	while(HAL_DMA_GetState( APP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    app_vision_load_to_txbuffer((uint8_t)pid_id, 0U);
    app_vision_load_to_txbuffer((float)p, 1U);
    app_vision_load_to_txbuffer((float)i, 5U);
    app_vision_load_to_txbuffer((float)d, 9U);
    app_vision_SendTxbuffer(CMD_READ_PID);
}



/**
 * @brief 接收并处理 CMD_READ_PID 命令的函数
 * 
 * @param     Vision_Rxbuffer 待处理的缓存
 */
void CMD_READ_PID_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_READ_PID)
    {
        float p,i,d;
		uint8_t pid_id = Vision_Rxbuffer[2];
        if( pid_id < __app_vision_pid_id_count )   // 验证参数有效
        {
            if( CMD_READ_PID_Rx_GetPidCallback(pid_id,&p,&i,&d) == HAL_OK ) // 读取，并确认读取正常否
            {
                CMD_READ_PID_Tx(Vision_Rxbuffer[2],p,i,d);  //发送
            }
        }
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
    CMD_READ_PID_Rx(Vision_Rxbuffer);
	APP_TEST_Rx(Vision_Rxbuffer);
}
///视觉串口发送函数
void CMD_GET_MCU_STATE_Tx(float pitch,float yaw_mech,float yaw_soft,uint8_t cloud_mode,uint8_t shoot_mode)
{
	while(HAL_DMA_GetState( APP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    // app_vision_load_to_txbuffer(cloud_mode, 0U);
    app_vision_load_to_txbuffer(pitch, 0U);
    app_vision_load_to_txbuffer(yaw_soft, 4U);
    app_vision_load_to_txbuffer(yaw_mech, 8U);
    // app_vision_load_to_txbuffer(shoot_mode, 13U);
    app_vision_SendTxbuffer(CMD_GET_MCU_STATE);
}
///串口发送到小主机日志系统
void ROBOT_ERR_Tx(uint8_t err_code)
{
	while(HAL_DMA_GetState( APP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    app_vision_load_to_txbuffer(err_code, 0U);
    app_vision_SendTxbuffer(ROBOT_ERR);
}
///发送底盘状态
void STA_CHASSIS_Tx(uint8_t chassis_mode,uint8_t pillar_flag,float velocity,float position)
{
		while(HAL_DMA_GetState( APP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
    app_vision_load_to_txbuffer(chassis_mode, 0U);
    app_vision_load_to_txbuffer(pillar_flag, 1U);
    app_vision_load_to_txbuffer(velocity, 2);
	app_vision_load_to_txbuffer(position, 6);
    app_vision_SendTxbuffer(STA_CHASSIS);
}



void JUD_GAME_STATUS_Tx(uint8_t game_progress,uint16_t stage_remain_time )
{
    while(HAL_DMA_GetState( APP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}
    
    memset(Vision_Txbuffer, 0, 18); //发送之前先清空一次
}
void JUD_ENY_HP_Tx(uint16_t hp);
void JUD_GAME_EVENT_Tx();      // 待定
void JUD_SELF_HP_Tx(uint16_t hp);         
void JUD_GUN_CHASSIS_HEAT_Tx(float chassis_power,uint16_t cha_pwr_buf,uint16_t gun_heat);
void JUD_SELF_BUFF_Tx(uint8_t buff_code);
void JUD_TAKING_DMG_Tx(uint8_t armor_id_enum,uint8_t hurt_type_enum);
void JUD_SHOOTING_Tx(uint8_t bullet_freq, float bullet_speed);
void JUD_AMMO_LEFT_Tx(uint16_t bulelt_left);

int v_test_id;
int v_test_size;
int v_test_cnt;

int v_in_id,v_in_cnt,v_in_max;

void APP_TEST_Rx(uint8_t *RxXer)
{
    if (RxXer[Function_word] == APP_TEST)
    {
		int id;
		memcpy(&id,&RxXer[2],4);
		VisionRx.id_got = id;
//		APP_TEST_Tx(id,100);
	}
}
void APP_TEST_Tx(int id,int size)
{
	memcpy(Vision_Txbuffer + 2, &id, 4);
	memcpy(Vision_Txbuffer + 6, &size, 4);
	app_vision_SendTxbuffer(APP_TEST);
}
void app_vision_test(int id, int size)
{
	if(id != v_test_id)
	{
		v_test_id = id;
		v_test_cnt = 0;
	}
	if(v_test_cnt == v_test_size)
		v_test_cnt = 0;
	APP_TEST_Tx(id,size);
}





























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
//HAL_StatusTypeDef app_vision_SendData(uint8_t _Functionword)
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

//    return HAL_UART_Transmit_DMA(&APP_VISION_UART, Vision_Txbuffer, 18);
//}
