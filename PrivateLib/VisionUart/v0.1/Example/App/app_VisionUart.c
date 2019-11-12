/** 
    * @brief    小主机通讯库，应用层
    * @details  
    * @author   Thunderdoge
    * @date      2019/11/12
    * @version  v0.1
    * @par Copyright (c):  OnePointFive, UESTC, 2019~2020
        OnePointFIve, the UESTC RoboMaster Team.
    */
#include "app_VisionUart.h"

//----------------------------------------------用户需要实现的函数---------------------------------------
int user_get_absolute_pitch(void)
{
    return 0; //请根据需要实现
}
uint8_t user_get_mcu_state(void)
{
    return 0; //请根据需要实现
}

//-------------------------------------------------Pack层函数---------------------------------------------
uint8_t Checkin_buf[UNPACK_BUF_LENGTH];  //接收到的帧暂存数组
uint8_t recv_buffer_[UNPACK_BUF_LENGTH]; //解析队列数组
int16_t recv_buf_len_;                   //队列长度
float RcvRate = 0.0f;

VisionInfoTypeDef VisionInfo; //解析到的消息储存结构体

/**
* @brief  		计算已打包数据帧的 和检验
* @param[in]  buffer_	帧的地址。buffer_已加入帧头和功能字。
* @retval  		和检验的值
*/
uint8_t static get_check_sum(uint8_t *buffer_)
{
    int check_sum = 0;
    for (int i = 0; i < len_ - 2; ++i)
    {
        check_sum += buffer_[i];
    }
    check_sum += 0x0d;
    return (unsigned char)(check_sum % 0xff);
}

/**
* @brief  检查收到的帧和检验是否正确
* @param[in]  buffer_	接收到的帧的地址
* @retval  1:和检验正确;	2:和检验错误.
*/
int static check_sum_noerror(uint8_t *buffer_)
{
    //    uint8_t no_error = 1;
    int check_sum = 0;
    for (int i = 0; i < len_ - 2; ++i)
    {
        check_sum += buffer_[i];
    }
    check_sum += 0x0d;
    return ((unsigned char)(check_sum % 0xff) == buffer_[len_ - 2]);
}

/**
* @brief  打包，即给数据帧加上帧头帧尾
* @param[in]  frame	未打包的帧结构体的地址
*/
void static pack(Dataframe_t *frame)
{
    frame->FrameHead = 0xFF;
    frame->SumCheck = get_check_sum((uint8_t *)frame);
    frame->FrameTail = 0x0D;
}
/**
 * @brief   向帧当中加载数据
 */
void static load(Dataframe_t *frame, uint8_t *goods, uint8_t loadPoint, uint8_t len)
{
    memcpy(&frame->Data[loadPoint], goods, len);
}

/**
* @brief  发送打包好的数据
* @param[in]  frame	待发送帧的地址
* @retval  发送情况值 HAL_StatusTypeDef
*/
HAL_StatusTypeDef sendPackedFrame(Dataframe_t *frame)
{
    return bsp_VisionUart_Send((uint8_t *)frame);
}
/**
 * @brief   发送受污染的打包好的数据包，仅供调试用
 * @param[in]   frame   打包好的帧的地址
 */
HAL_StatusTypeDef static sendPackedFrameNoisy(Dataframe_t *frame, uint8_t noisyBitHead, uint8_t noisyBitTail)
{
    uint8_t tmpTx[100];
    int i;
    srand(HAL_GetTick());
    for (i = 0; i < noisyBitHead; i++)
    {
        tmpTx[i] = rand() % 0xFF;
    }
    for (i = 0; i < noisyBitTail; i++)
    {
        tmpTx[i + noisyBitHead + DATA_FRAME_LENGTH] = rand() % 0xFF;
    }
    memcpy(tmpTx + noisyBitHead, (uint8_t *)frame, DATA_FRAME_LENGTH);
    return HAL_UART_Transmit(&VISION_HUART, tmpTx, (noisyBitHead + DATA_FRAME_LENGTH + noisyBitTail), 0XFFFF);
}
/**
 * @brief  解包收 到的数据
 * @param[in]  tmp_buffer	收到的数据的缓存的地址
 * @param[in]  recv_len	缓存长度
 * @retval  1:接收正常; 0:接收和和检验错误.
 */
int static unPack(unsigned char *tmp_buffer, int recv_len) //解包tmp_buffer，接收长度rec_len
{
    if (recv_len == len_ && tmp_buffer[0] == 0xff && tmp_buffer[len_ - 1] == 0x0d)
    { //如果符合格式，直接解包
        memcpy(Checkin_buf, tmp_buffer, len_);
        recv_buf_len_ = 0;
    }
    if (recv_len + recv_buf_len_ > UNPACK_BUF_LENGTH)
    {                      //如果超出解包缓存处理长度
        recv_buf_len_ = 0; //之前的解包缓存清零
    }
    memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer, recv_len); //将 tmp_buffer 加入到 解包缓冲数组recv_buffer_中
    recv_buf_len_ += recv_len;                                  //缓冲数组下标增长

    for (int start_point = 0; (start_point + len_) <= recv_buf_len_; start_point++)
    { //窗口滑动扫描符合帧头帧尾格式的
        if (recv_buffer_[start_point] == 0xff && recv_buffer_[start_point + len_ - 1] == 0x0d)
        {
            memcpy(Checkin_buf, recv_buffer_ + start_point, len_); //若符合，加入到待解包缓存 Checkin_buf 中
            int k = 0;
            for (int j = start_point + len_; j < recv_buf_len_; j++)
            { //解析后将后面未解析的部分移动到recv_buffer_头部
                recv_buffer_[k] = recv_buffer_[j];
                k++;
            }
            recv_buf_len_ = k; //恢复缓冲数组下标增长
            break;
        }
    }
    if (check_sum_noerror(Checkin_buf))
        return 1;
    else
        return 0;
}
/**
 * @brief  加载收到的数据
 * @param[in]  buffer_	待加载的帧的地址
 * @retval  0: 正常;
*/

int static unLoad(unsigned char *buffer_)
{
    Dataframe_t *frame_t = (Dataframe_t *)buffer_;      //强转指针类型
    VisionInfo.LastFuntionWord = frame_t->FunctionWord; //加载功能字
    switch (frame_t->FunctionWord)
    {
    case CMD_GIMBAL_RELATIVE_CONTROL:
        memcpy(&(VisionInfo.GimbolRelative_Ctrl), frame_t->Data, sizeof(GimbolRelative_Rx));
        break;
    case CMD_GIMBAL_ABSOLUTE_CONTROL:
        memcpy(&(VisionInfo.GimbolAbsolute_Ctrl), frame_t->Data, sizeof(GimbolAbsolute_Rx));
        break;
    case CMD_SHOOT:
        memcpy(&(VisionInfo.Shooting_Ctrl), frame_t->Data, sizeof(Shooting_Rx));
        break;
    case CMD_TXRX_TEST:
        memcpy(&(VisionInfo.TxRxTestMsg), frame_t->Data, sizeof(TransmitTest_Tx));
        break;

    default:
        break;
    }
    return 0;
} /**
* @brief  
* @details  
* @param[in]  
* @retval  
*/

static void app_VisionUart_TxTestRx(int test_cnt, int rcv_val)
{
    static int gTestCnt = 0;
    static int gRcvCnt = 0;
    static int gRcvVal = 0;

    gTestCnt = test_cnt;
    if (rcv_val > gRcvVal)
    {
        gRcvCnt++;
    }
    if (rcv_val < gRcvVal)
    {
        RcvRate = (float)gRcvCnt / (float)gTestCnt;
        gRcvCnt = 0;
    }
    gRcvVal = rcv_val;
}

//-------------------------------------------Interface层函数----------------------------------------
/**	
 * @brief 串口信息应答
*/
HAL_StatusTypeDef app_VisionUart_AckMsg(void)
{
    switch (VisionInfo.LastFuntionWord)
    {
    case CMD_GET_GIMBAL_ABSOLUTE_ANGLE:
        return app_VisionUart_Send_gimbal_Absolute(user_get_absolute_pitch());
        break;
    case CMD_GET_MCU_STATE:
        return app_VisionUart_Send_mcu_state(user_get_mcu_state());
        break;
    case CMD_TXRX_TEST:
        app_VisionUart_TxTestRx(VisionInfo.TxRxTestMsg.Msg_Cnt_Total,
                                VisionInfo.TxRxTestMsg.Tx_Value);
        return HAL_OK;
        break;

    default:
        return HAL_ERROR; //如果全部不匹配
        break;
    }
    VisionInfo.LastFuntionWord = 0x00; //暂存的功能字清零
    return HAL_ERROR;
}

/**
* @brief  	接收数据帧
* @details  调用了unPack, unLoad, 解析好的数据放到 VisionInfo 中
* @retval  	执行状况
*/
HAL_StatusTypeDef app_VisionUart_RecvPack(void)
{
    int ret_len = VISION_REC_LENGTH;
    unsigned char tmp_buffer[VISION_REC_LENGTH];
    bsp_VisionUart_Receive(tmp_buffer);
    if (unPack(tmp_buffer, ret_len))
    { // check packet pass
        unLoad(Checkin_buf);
        memset((void *)Checkin_buf, 0, UNPACK_BUF_LENGTH);
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}
/**
 * @brief 打包并发送数据Data
 * @param[in]   Data    数据的指针
 * @param[in]   FunctionWord    功能字
 */ 
HAL_StatusTypeDef app_VisionUart_SendPack(uint8_t *Data, uint8_t FunctionWord)
{
    Dataframe_t frame;
    frame.FunctionWord = FunctionWord;
    load(&frame, Data, 0, DATA_FRAME_LENGTH);
    pack(&frame);
    return sendPackedFrame(&frame);
}
///**   暂时未实现的函数，将在以后的版本中实现
// * @brief				发送云台相对角
// * @retval  		执行状况
// */
//HAL_StatusTypeDef app_VisionUart_Send_gimbal_Relative(float pitch, float yaw, unsigned char gimbal_speed_mode, unsigned char shoot_mode)
//{
//	GimbolRelative_Rx tmpTx;
//	tmpTx.pitch = pitch;
//	tmpTx.yaw 	= yaw;
//	tmpTx.
//	return app_VisionUart_SendPack( (uint8_t*)&tmpTx , CMD_GIMBAL_RELATIVE_CONTROL );
//}
HAL_StatusTypeDef app_VisionUart_Send_gimbal_Absolute(float pitch)
{
    //    Dataframe_t frame;
    GimbolAbsolute_Tx tmpTx;
    tmpTx.pitch = pitch;
    //		frame.FunctionWord = CMD_GET_GIMBAL_ABSOLUTE_ANGLE;
    //    load(&frame , (uint8_t*)&tmpTx , 0 , sizeof(tmpTx) );
    //    pack(&frame);
    //    return sendPackedFrame(&frame);
    return app_VisionUart_SendPack((uint8_t *)&tmpTx, CMD_GET_GIMBAL_ABSOLUTE_ANGLE);
}
HAL_StatusTypeDef app_VisionUart_Send_mcu_state(uint8_t mode)
{
    //    Dataframe_t frame;
    mcuStatus_Tx tmpTx;
    //		frame.FunctionWord = CMD_GET_MCU_STATE;
    tmpTx.mode = mode;
    //    load(&frame , (uint8_t*)&tmpTx , 0 ,sizeof(tmpTx) );
    //    pack(&frame);
    //    return sendPackedFrame(&frame);
    return app_VisionUart_SendPack((uint8_t *)&tmpTx, CMD_GET_MCU_STATE);
}
void app_VisionUart_TxTest(int32_t TxMsgCnt)
{
    int i;
    Dataframe_t frame;
    TransmitTest_Tx tmpTx;
    tmpTx.Msg_Cnt_Total = TxMsgCnt;
    for (i = 0; i < tmpTx.Msg_Cnt_Total; i++)
    {
        tmpTx.Tx_Value = i;
        frame.FunctionWord = CMD_TXRX_TEST;
        load(&frame, (uint8_t *)&tmpTx, 0, sizeof(tmpTx));
        pack(&frame);
        sendPackedFrameNoisy(&frame, rand() % 9, rand() % 9);
        osDelay(1);
    }
}

//  global variables
