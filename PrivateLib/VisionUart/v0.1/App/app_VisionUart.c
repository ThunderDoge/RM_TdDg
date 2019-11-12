/** 
    * @brief    С����ͨѶ�⣬Ӧ�ò�
    * @details  
    * @author   Thunderdoge
    * @date      2019/11/12
    * @version  v0.1
    * @par Copyright (c):  OnePointFive, UESTC, 2019~2020
        OnePointFIve, the UESTC RoboMaster Team.
    */
#include "app_VisionUart.h"

//----------------------------------------------�û���Ҫʵ�ֵĺ���---------------------------------------
int user_get_absolute_pitch(void)
{
    return 0; //�������Ҫʵ��
}
uint8_t user_get_mcu_state(void)
{
    return 0; //�������Ҫʵ��
}

//-------------------------------------------------Pack�㺯��---------------------------------------------
uint8_t Checkin_buf[UNPACK_BUF_LENGTH];  //���յ���֡�ݴ�����
uint8_t recv_buffer_[UNPACK_BUF_LENGTH]; //������������
int16_t recv_buf_len_;                   //���г���
float RcvRate = 0.0f;

VisionInfoTypeDef VisionInfo; //����������Ϣ����ṹ��

/**
* @brief  		�����Ѵ������֡�� �ͼ���
* @param[in]  buffer_	֡�ĵ�ַ��buffer_�Ѽ���֡ͷ�͹����֡�
* @retval  		�ͼ����ֵ
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
* @brief  ����յ���֡�ͼ����Ƿ���ȷ
* @param[in]  buffer_	���յ���֡�ĵ�ַ
* @retval  1:�ͼ�����ȷ;	2:�ͼ������.
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
* @brief  �������������֡����֡ͷ֡β
* @param[in]  frame	δ�����֡�ṹ��ĵ�ַ
*/
void static pack(Dataframe_t *frame)
{
    frame->FrameHead = 0xFF;
    frame->SumCheck = get_check_sum((uint8_t *)frame);
    frame->FrameTail = 0x0D;
}
/**
 * @brief   ��֡���м�������
 */
void static load(Dataframe_t *frame, uint8_t *goods, uint8_t loadPoint, uint8_t len)
{
    memcpy(&frame->Data[loadPoint], goods, len);
}

/**
* @brief  ���ʹ���õ�����
* @param[in]  frame	������֡�ĵ�ַ
* @retval  �������ֵ HAL_StatusTypeDef
*/
HAL_StatusTypeDef sendPackedFrame(Dataframe_t *frame)
{
    return bsp_VisionUart_Send((uint8_t *)frame);
}
/**
 * @brief   ��������Ⱦ�Ĵ���õ����ݰ�������������
 * @param[in]   frame   ����õ�֡�ĵ�ַ
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
 * @brief  ����� ��������
 * @param[in]  tmp_buffer	�յ������ݵĻ���ĵ�ַ
 * @param[in]  recv_len	���泤��
 * @retval  1:��������; 0:���պͺͼ������.
 */
int static unPack(unsigned char *tmp_buffer, int recv_len) //���tmp_buffer�����ճ���rec_len
{
    if (recv_len == len_ && tmp_buffer[0] == 0xff && tmp_buffer[len_ - 1] == 0x0d)
    { //������ϸ�ʽ��ֱ�ӽ��
        memcpy(Checkin_buf, tmp_buffer, len_);
        recv_buf_len_ = 0;
    }
    if (recv_len + recv_buf_len_ > UNPACK_BUF_LENGTH)
    {                      //�������������洦����
        recv_buf_len_ = 0; //֮ǰ�Ľ����������
    }
    memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer, recv_len); //�� tmp_buffer ���뵽 �����������recv_buffer_��
    recv_buf_len_ += recv_len;                                  //���������±�����

    for (int start_point = 0; (start_point + len_) <= recv_buf_len_; start_point++)
    { //���ڻ���ɨ�����֡ͷ֡β��ʽ��
        if (recv_buffer_[start_point] == 0xff && recv_buffer_[start_point + len_ - 1] == 0x0d)
        {
            memcpy(Checkin_buf, recv_buffer_ + start_point, len_); //�����ϣ����뵽��������� Checkin_buf ��
            int k = 0;
            for (int j = start_point + len_; j < recv_buf_len_; j++)
            { //�����󽫺���δ�����Ĳ����ƶ���recv_buffer_ͷ��
                recv_buffer_[k] = recv_buffer_[j];
                k++;
            }
            recv_buf_len_ = k; //�ָ����������±�����
            break;
        }
    }
    if (check_sum_noerror(Checkin_buf))
        return 1;
    else
        return 0;
}
/**
 * @brief  �����յ�������
 * @param[in]  buffer_	�����ص�֡�ĵ�ַ
 * @retval  0: ����;
*/

int static unLoad(unsigned char *buffer_)
{
    Dataframe_t *frame_t = (Dataframe_t *)buffer_;      //ǿתָ������
    VisionInfo.LastFuntionWord = frame_t->FunctionWord; //���ع�����
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

//-------------------------------------------Interface�㺯��----------------------------------------
/**	
 * @brief ������ϢӦ��
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
        return HAL_ERROR; //���ȫ����ƥ��
        break;
    }
    VisionInfo.LastFuntionWord = 0x00; //�ݴ�Ĺ���������
    return HAL_ERROR;
}

/**
* @brief  	��������֡
* @details  ������unPack, unLoad, �����õ����ݷŵ� VisionInfo ��
* @retval  	ִ��״��
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
 * @brief �������������Data
 * @param[in]   Data    ���ݵ�ָ��
 * @param[in]   FunctionWord    ������
 */ 
HAL_StatusTypeDef app_VisionUart_SendPack(uint8_t *Data, uint8_t FunctionWord)
{
    Dataframe_t frame;
    frame.FunctionWord = FunctionWord;
    load(&frame, Data, 0, DATA_FRAME_LENGTH);
    pack(&frame);
    return sendPackedFrame(&frame);
}
///**   ��ʱδʵ�ֵĺ����������Ժ�İ汾��ʵ��
// * @brief				������̨��Խ�
// * @retval  		ִ��״��
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
