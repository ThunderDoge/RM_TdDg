/** 
 * @file: app_vision.cpp 
 * @encoding
* @brief    �Ӿ�����Э���ļ�
* @details  32��С������ͨѶ����
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020���
* @par ��־
    2019/12/8   ��Evan-GH���˴��ڰ汾1.2�ϲ�����������⴮��v2.1.0
    2019/12/8   ThunderDoge�����Լ���ϲ�ý����ڿ����Ϊv2.1.1   �����˲��ֹ��ܡ��޸���һЩд��

    PS ���Ӿ���ͨ��Э��μ���RM2020�����Ӿ�Э�� v2.0�� by Evan-GH
*/



///�������ļ�
#include "app_vision.hpp"
//#include "SentryCommu.hpp"



//�����ñ�־λ
#define VIUART_DISABLE_OTHER_IT
#define _VISION_DEBUG



///��������ʹ�õĽṹ��
sentry_vision_data VisionRx, VisionTx; ///�Ӿ����ڽ�����������,�Ӿ����ڷ��͵�����
uint8_t Vision_Rxbuffer[BSP_VISION_BUFFER_SIZE] = {0};     ///���ڽ������ݻ������飬���ڻ�������������������֡������



///�ؼ�״̬����
static int8_t Array_index = 0;                             ///���������ݼ����ָ��
int not_analysed_index=0;                  /// Vision_Rxbuffer�д�[0]��[not_analysed_index -1]�����ݶ�δ��������



/// ����Э������ĺ����ļ��ϡ��ڻ������ݽ����е��á�
void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer);   ///ͳһ�Ľ��ջ��洦��������Vision_RxbufferӦ����֤ͨ���ѷ�������֡֡��ʽ

/**
* @brief  �Ӿ����ڽ�������
*   ������յ���ԭʼ����ת���ɿ�ʹ�õ�����
*   �ؼ�����
*   unsigned char _recv_buf[_len*3];  //����ʱ���ڴ洢ԭʼ����
*   int _recv_len;                //���ջ�������ǰ��Ч�ֽ�����
*   �洢�ṹ����,Խ���յ����ֽ���Խǰ��
*   ������ά���߼�
*   1.�յ���ȷ֡,��ȷ֡����ȷ֮֡ǰ���������,���������������ǰ�ƶ�
*   2.��������������,�����ǰ���_len���ֽ�,�����ֽ�������ǰ�ƶ�_len���ֽ�,���յ�������׷���ں�
*   3.֡ͷ֡β����,У��Ͳ�����,�������¼���
*   4.��_recv_buf��ֻ��ǰ_recv_len�ֽ���Ч,ʣ��Ļ�����������Ч,������ά��,��ֹ��ȡ
* @details  ���ж��У��Ի���������ݽ���һ�α�������������
* @param  NULL
* @retval  uint8_t 1��û�н������������������ 0�����Ѿ����
*/
uint8_t app_vision_Analysis(void)
{
    int16_t _check_sum = 0;                         //��У���ñ���
    if (Array_index <= BSP_VISION_BUFFER_SIZE - 18) //���δ�����±�
    {
        //��������������֡������Ҫ֡ͷ֡β���˲ſ�ʼ���
        if ((Vision_Rxbuffer[Array_index + Frame_header] == FRAME_HEADER_DATA) && (Vision_Rxbuffer[Array_index + Frame_end] == FRAME_END_DATA))
        {
            //֡ͷ֡β���ˣ����һ�κ�У��
            for (int i = 0; i < 18; i++)
            {
                if (i != Sum_check)
                    _check_sum += Vision_Rxbuffer[Array_index + i];
            }
            _check_sum = _check_sum & 0xff;
            if (_check_sum != Vision_Rxbuffer[Array_index + Sum_check]) //����У��
            {
                Array_index += 1; //��У����ˣ�ָ���ƶ�1λ�������¼��
                return 1;         //У��ͳ���ֱ���˳�,�����ڻ������������֡
            }

            Array_index += 18; //����֡һ֡����Ϊ18�������ƶ�18λ
            return 1;          //������һ֡���ƶ�18λ�������
        }
        else
        {
            Array_index += 1; //֡ͷ֡β���ԣ��ƶ�һλ�������
            return 1;   //����1��ʾ����δ��ɣ�Ҫ�����ѭ������
        }
    }
    else //���µ��������Բ��ܹ���һ֡�ˣ���������
    {
        Array_index = 0; //��ָ�븴λ
        return 0;        //�Ѿ�����һ�黺��������������
    }
}




/**
* @brief  �Ӿ����ڽ�������
*   ������յ���ԭʼ����ת���ɿ�ʹ�õ�����
*   �ؼ�����
*   unsigned char _recv_buf[_len*3];  //����ʱ���ڴ洢ԭʼ����
*   int _recv_len;                //���ջ�������ǰ��Ч�ֽ�����
*   �洢�ṹ����,Խ���յ����ֽ���Խǰ��
*   ������ά���߼�
*   1.�յ���ȷ֡,��ȷ֡����ȷ֮֡ǰ���������,���������������ǰ�ƶ�
*   2.��������������,�����ǰ���_len���ֽ�,�����ֽ�������ǰ�ƶ�_len���ֽ�,���յ�������׷���ں�
*   3.֡ͷ֡β����,У��Ͳ�����,�������¼���
*   4.��_recv_buf��ֻ��ǰ_recv_len�ֽ���Ч,ʣ��Ļ�����������Ч,������ά��,��ֹ��ȡ
* @details  ���ж��У��Ի���������ݽ���һ�α�������������
* @param  NULL
* @retval  uint8_t 1��û�н������������������ 0�����Ѿ����
*/
uint8_t app_vision_analysis_intgrated(void)
{
    int16_t _check_sum = 0;                         //��У���ñ���
    
    uint32_t head_index=0;            //ָʾĿǰ���ڽ����Ķε���ʼ��
    uint32_t end_index=Frame_end;            //ָʾĿǰ���ڽ����ĶεĽ�����
	
	int first_0xFF=-1;		// -1 �³�ʼֵ,��ʾû��

    uint8_t frame_solved=0;

    if( not_analysed_index-1 < end_index )   // �����յ������ݸ�����������һ��֡
    {   return 0;   }

    // ���� Vision_Rxbuffer
    for(;
    end_index < not_analysed_index;    // ���δ�������յ������ݵ��±�
    head_index++ , end_index++) // ָʾ��������
    {
		if( Vision_Rxbuffer [ head_index ] == FRAME_HEADER_DATA )   // ��0XFF
		{
            if(Vision_Rxbuffer [ end_index ] == FRAME_END_DATA)     // ֡β�� 0X0D
            {

                // ����У���
                for (int i = head_index; i <= end_index; i++)   
                {
                    if (i != head_index+Sum_check)
                        _check_sum += Vision_Rxbuffer[i];
                    _check_sum &= 0xff;
                }


                // ���У��ͷ���
                if (_check_sum == Vision_Rxbuffer[ head_index + Sum_check ] ) // ��У�����
                {
                    //֡ͷ֡β��ȷ����У����ȷ����ʼ����
                    VisionRx.Ready_flag ++; //������ݾ���
                    frame_solved++;          //�����֡�ļ���
                    SentryVisionUartRxAll(Vision_Rxbuffer + head_index);   //��������֡
                    //���е�����ͱ�ʾ�����Ѿ��ɹ���һ֡��Ч�����Ѿ����
                    head_index = end_index;
                    end_index += Frame_end;

                    // ���֮ǰ����� first_0xFF��Ҫ������
                    first_0xFF = -1;
                }
                else
                {
                    continue;   //��������һ��
                }

            }
            else    // ����֡ͷ���ǲ�����֡β
            {
                if(first_0xFF == -1)    // ��¼��һ�� 0XFF
                {
                    first_0xFF = head_index;
                }
            }
		}
        else    // ���� 0XFF
        {
            continue;   //��������һ��
        }
    }
    // �������˽���

    // ��δ�����������Ƶ�ǰ��
	
	// ��һ��0XFF ������ ��������һ��֡�ĳ�������.���������һ��.
	if(first_0xFF == 0)
	{
		for(int i=head_index;i<not_analysed_index;i++)
		{
			if(Vision_Rxbuffer[i] == 0xff)
			{
				first_0xFF = i;
				break;		// �ҵ�������
			}
		}
	}
	//����û��
	if(first_0xFF == -1)
	{
		first_0xFF = not_analysed_index;	// ����˵������û��0XFF,ֱ���ӵ�����
	}
	
	// �ƶ����ݵ�����ͷ�������ǵ�һ��0XFF֮ǰ�Ĳ�Ҫ
    int from_index,to_index;
    for(from_index =first_0xFF,to_index=0;
        from_index < not_analysed_index;
        from_index++,to_index++)
    {
        Vision_Rxbuffer[to_index] = Vision_Rxbuffer[from_index];
    }
    not_analysed_index = to_index;


    // ���ؽ�������֡��������
    return frame_solved;

}



/**
* @brief  �Ӿ����ڳ�ʼ��
* @details  ���ÿ���(IDLE)�ж�λ�����������жϡ��ر��������޹��жϡ�����UART-DMA���գ��󶨵�Vision_Rxbuffer����������ΪBSP_VISION_BUFFER_SIZE��
* @param  NULL
* @retval  NULL
*/
void app_vision_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);          //��������ж�λ
    __HAL_UART_ENABLE_IT(&BSP_VISION_UART, UART_IT_IDLE); //ʹ��DMA���տ����ж�

#ifdef VIUART_DISABLE_OTHER_IT //�����������е��ж�
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_CTS);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_LBD);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_TC);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&BSP_VISION_UART, UART_IT_ERR);
#endif

    HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //��ʼDMA���գ�DMA���ӵ�Vision_Rxbuffer
}








/**
* @brief  �Ӿ������жϴ�����
* @details  ���ڶ��Ӿ����͹����������������������������һ�λ����
* @param  NULL
* @retval  NULL
*/
void app_vision_It(void)
{
#ifndef VIUART_DISABLE_OTHER_IT                                         //�����������е��жϣ�������Ҫ�ж�
    if (__HAL_UART_GET_FLAG(&BSP_VISION_UART, UART_FLAG_IDLE) != RESET) //��������˿����ж�
#endif
    {
        HAL_UART_DMAStop(&BSP_VISION_UART); //�ر�DMA
        while (app_vision_Analysis())
            ;                                                                                       //���ݽ���������һ�λ�����
        memset(Vision_Rxbuffer, 0, BSP_VISION_BUFFER_SIZE);                                         //������ɣ�������0
        __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //��������жϱ�־λ
        HAL_UART_DMAResume(&BSP_VISION_UART);                                                       //���´�DMA
        HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //���¿���DMA���մ���
    }
}



/**
 * @brief ���յ�����
 * 
 */
void app_vision_dma_abort_in_idle(void)
{
    // �����յ������ݵǼǵ�δ��������
    not_analysed_index += BSP_VISION_UART.RxXferSize -BSP_VISION_UART.RxXferCount;
    
    HAL_UART_AbortReceive_IT(&BSP_VISION_UART); // ֹͣ�գ����������������DMA���պ��жϽ��յ�ֹͣ������˺���ע�͡�

    app_vision_analysis_intgrated();            //����. ������ not_analysed_index ������
    
    HAL_UART_Receive_DMA(&BSP_VISION_UART,      //��������
        Vision_Rxbuffer+not_analysed_index,       //not_analysed_indexer ����ʼд
        BSP_VISION_BUFFER_SIZE-not_analysed_index); // ���յ�����һֱ������ Vision_Rxbuffer
    
    return ;
}


/**
 * @brief С����ͨ�� �����ж�DMA������� �ص�������
 * �� HAL_UART_RxCpltCallback �е��ô˺�����ǰ��Ҫ���� if huart1��
 * 
 * ��ȷ����Ӧ�ý�������������Ż���ô˻ص�������
 * �������ݽ�����
 * 
 */
void app_vision_dma_cpltcallback(void)
{
	not_analysed_index = BSP_VISION_BUFFER_SIZE;	// ֱ���趨��������ΪΪδ��������ΪDMA/IT��������ʱ �趨�ǽ���������Ż������ɷ��ء�
	
    app_vision_analysis_intgrated();            //����
    
    HAL_UART_Receive_DMA(&BSP_VISION_UART,      //��������
        Vision_Rxbuffer+not_analysed_index,       //not_analysed_indexer ����ʼд
        BSP_VISION_BUFFER_SIZE-not_analysed_index+1); // ���յ�����һֱ������ Vision_Rxbuffer

    return ;
}














/**
* @brief  �Ӿ����ڷ��ͺ���
* @details  ������Ӿ����͵�����������־ϵͳ����Ҫʹ���߶Խṹ�������������
* @param  uint8_t _Functionword �������б�
* @retval  HAL_StatusTypeDef HAL_OK ���ͳɹ� HAL_ERROR����ʧ��
*/
uint8_t Vision_Txbuffer[18] = {0}; //����������
/**
 * @brief ������װ�뻺��Vision_Txbuffer��
 * 
 * @param     u8data ��һ�ֽڵ�����
 * @param     location_at_buffdata ������֡�����ݡ����е�λ�á�0��ʾ�����ݡ���1�ֽ�. ����
 */
void app_vision_load_to_txbuffer(uint8_t u8data, int location_at_buffdata)
{
    Vision_Txbuffer[location_at_buffdata + 2] = u8data;
}
/**
 * @brief 
 * 
 * @param     fdata ��װ��ĸ�������
 * @param     location_at_buffdata ������֡�����ݡ����е�λ�á�0��ʾװ�롾���ݡ���1~4�ֽ�. ����. ע��1��floatռ��4�ֽ�
 */
void app_vision_load_to_txbuffer(float fdata, int location_at_buffdata)
{
    //    *((float*)(&Vision_Txbuffer[location_at_buffdata+2])) = fdata;	This cause HardFault
    memcpy(Vision_Txbuffer + location_at_buffdata + 2, &fdata, 4);
}
/**
 * @brief ��������֡
 * 
 * @param     _Functionword ����������֡�Ĺ����֡�
 * @return HAL_StatusTypeDef HAL_OKΪ����
 */
HAL_StatusTypeDef app_vision_SendTxbuffer(uint8_t _Functionword)
{
    int16_t _check_sum = 0; //��У���ñ���
    // memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
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
//VisionUart TxRx Functions �Ӿ����ں���
//VisionUart Recv �Ӿ����ڽ��պ���
///��̨��ԽǶȿ���
void CMD_GIMBAL_RELATIVE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_RELATIVE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 2, 4); //��̨�ǶȽ���
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1); //��̨ģʽ����
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1); //���ģʽ
        VisionRx.cloud_ctrl_mode = relative_cloud;                  //���ݾ���
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///��̨���ԽǶȿ���
void CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_GIMBAL_ABSOLUTE_CONTROL)
    {
        VisionRx.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
        memcpy(&VisionRx.Yaw, Vision_Rxbuffer + 2, 4);
        memcpy(&VisionRx.Pitch, Vision_Rxbuffer + 6, 4);
        memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + 10, 1);
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 11, 1);
        VisionRx.cloud_ctrl_mode = absolute_cloud; //���ݾ���
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///��̨�ٶȿ���
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
 * @brief       ���������ִ�лص�����
 * ���µĲ����ľ��庬����Ҫ�ڶ��崦д��
 * ����Ӧ���޸Ĵ˶��塣��Ӧ������ʵ�ִ˺����Ĺ��ܣ���Ϊ�˴��Ķ�����WEAK��
 * @param     bullet_speed  �ӵ������ٶ�
 * @param     fire_freq     �ӵ����Ƶ��
 * @param     shoot_mode    �ӵ����ģʽ��
 */
__weak void CMD_SHOOT_ExecuteCallback(float bullet_speed, uint8_t fire_freq, uint8_t shoot_mode)
{
	// ����Ӧ���޸Ĵ˺�������Ӧ������ʵ�ִ˺����Ĺ��ܣ���Ϊ�˴��Ķ�����WEAK��
	UNUSED(bullet_speed);
	UNUSED(fire_freq);
	UNUSED(shoot_mode);
}

void CMD_SHOOT_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_SHOOT)
    {
        VisionRx.Function_word = CMD_SHOOT;
        VisionRx.Shoot_mode = 1;                               //���ָ�����
        memcpy(&VisionRx.Shoot_speed, Vision_Rxbuffer + 2, 4); //����ٶ�
        memcpy(&VisionRx.Shoot_freq, Vision_Rxbuffer + 6, 1);  //���Ƶ��
        memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + 7, 1);  //���ģʽ
        VisionRx.UpdateTime = HAL_GetTick();
		CMD_SHOOT_ExecuteCallback(VisionRx.Shoot_speed,VisionRx.Shoot_freq,VisionRx.Shoot_mode);
    }
}
///�����˶�����
void CMD_CHASSIS_CONTROL_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_CHASSIS_CONTROL)
    {
        VisionRx.Function_word = CMD_CHASSIS_CONTROL;
        memcpy(&VisionRx.Vx, Vision_Rxbuffer + 2, 4); //�����ٶȽ���
        memcpy(&VisionRx.Vy, Vision_Rxbuffer + 6, 4);
        VisionRx.UpdateTime = HAL_GetTick();
    }
}
///����·�̿���
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
///����·�̿���������
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
 * @brief       ��ȡPID�����Ļص�������������ʵ��
 * 
 * @param     pid_id    PID���
 * @param     p         ��ȡ��д��P�ĵ�ַ
 * @param     i         ��ȡ��д��I�ĵ�ַ
 * @param     d         ��ȡ��д��D�ĵ�ַ
 * @return ״ֵ̬����������HAL_OK, �쳣����HAL_ERROR 
 */
__weak HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d)
{
    //������������Լ���ʵ�֡�
    UNUSED(pid_id);
    UNUSED(p);
    UNUSED(i);
    UNUSED(d);
    return HAL_ERROR;
}



/**
 * @brief       ���� CMD_READ_PID �����ֵ�ĺ���
 * 
 * @param     pid_id    PID���
 * @param     p         ��ȡ����P
 * @param     i         ��ȡ����I
 * @param     d         ��ȡ����D
 */
void CMD_READ_PID_Tx(uint8_t pid_id,float p,float i,float d)
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
    app_vision_load_to_txbuffer((uint8_t)pid_id, 0U);
    app_vision_load_to_txbuffer((float)p, 1U);
    app_vision_load_to_txbuffer((float)i, 5U);
    app_vision_load_to_txbuffer((float)d, 9U);
    app_vision_SendTxbuffer(CMD_READ_PID);
}



/**
 * @brief ���ղ����� CMD_READ_PID ����ĺ���
 * 
 * @param     Vision_Rxbuffer ������Ļ���
 */
void CMD_READ_PID_Rx(uint8_t *Vision_Rxbuffer)
{
    if (Vision_Rxbuffer[Function_word] == CMD_READ_PID)
    {
        float p,i,d;
		uint8_t pid_id = Vision_Rxbuffer[2];
        if( pid_id < __app_vision_pid_id_count )   // ��֤������Ч
        {
            if( CMD_READ_PID_Rx_GetPidCallback(pid_id,&p,&i,&d) == HAL_OK ) // ��ȡ����ȷ�϶�ȡ������
            {
                CMD_READ_PID_Tx(Vision_Rxbuffer[2],p,i,d);  //����
            }
        }
    }
}















///ȫ�������	�����Ĺ����ֽ��պ����������������
///�Ӿ������жϽ��պ���
static void SentryVisionUartRxAll(uint8_t *Vision_Rxbuffer)
{
    CMD_GIMBAL_RELATIVE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_GIMBAL_ABSOLUTE_CONTROL_Rx(Vision_Rxbuffer);
    CMD_SHOOT_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOACTION_CONTROL_Rx(Vision_Rxbuffer);
    CMD_CHASSIS_LOCATION_LIMIT_SPEED_Rx(Vision_Rxbuffer);
	CMD_GIMBAL_SPEED_CONTROL_Rx(Vision_Rxbuffer);
    CMD_READ_PID_Rx(Vision_Rxbuffer);
}
///�Ӿ����ڷ��ͺ���
void CMD_GET_MCU_STATE_Tx(float pitch,float yaw_mech,float yaw_soft,uint8_t cloud_mode,uint8_t shoot_mode)
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
    app_vision_load_to_txbuffer(cloud_mode, 0U);
    app_vision_load_to_txbuffer(pitch, 1);
    app_vision_load_to_txbuffer(yaw_mech, 5);
    app_vision_load_to_txbuffer(yaw_soft, 9U);
    app_vision_load_to_txbuffer(shoot_mode, 13U);
    app_vision_SendTxbuffer(CMD_GET_MCU_STATE);
}
///���ڷ��͵�С������־ϵͳ
void ROBOT_ERR_Tx(uint8_t err_code)
{
	while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
    app_vision_load_to_txbuffer(err_code, 0U);
    app_vision_SendTxbuffer(ROBOT_ERR);
}
///���͵���״̬
void STA_CHASSIS_Tx(uint8_t chassis_mode,uint8_t pillar_flag,float velocity,float position)
{
		while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}

    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
    app_vision_load_to_txbuffer(chassis_mode, 0U);
    app_vision_load_to_txbuffer(pillar_flag, 1U);
    app_vision_load_to_txbuffer(velocity, 2);
	app_vision_load_to_txbuffer(position, 6);
    app_vision_SendTxbuffer(STA_CHASSIS);
}



void JUD_GAME_STATUS_Tx(uint8_t game_progress,uint16_t stage_remain_time )
{
    while(HAL_DMA_GetState( BSP_VISION_UART.hdmatx )!=HAL_DMA_STATE_READY)
	{
//		vTaskDelay(1);
	}
    
    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
}
void JUD_ENY_HP_Tx(uint16_t hp);
void JUD_GAME_EVENT_Tx();      // ����
void JUD_SELF_HP_Tx(uint16_t hp);         
void JUD_GUN_CHASSIS_HEAT_Tx(float chassis_power,uint16_t cha_pwr_buf,uint16_t gun_heat);
void JUD_SELF_BUFF_Tx(uint8_t buff_code);
void JUD_TAKING_DMG_Tx(uint8_t armor_id_enum,uint8_t hurt_type_enum);
void JUD_SHOOTING_Tx(uint8_t bullet_freq, float bullet_speed);
void JUD_AMMO_LEFT_Tx(uint16_t bulelt_left);




























#endif //USE_VISION


/**
  * @brief ����֡���Ա����
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
  * @brief  ��̨��ԽǶȿ���
  * @details  
  */
//void CMD_GIMBAL_RELATIVE_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//    memcpy(&VisionRx.Pitch, Vision_Rxbuffer + Array_index + 2, 4); //��̨�ǶȽ���
//    memcpy(&VisionRx.Yaw, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1); //��̨ģʽ����
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1); //���ģʽ
//    VisionRx.Ready_flag = 1;                                             //���ݾ���
//}
///**
//  * @brief  ��̨���ԽǶȿ���
//  */
//void CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//    memcpy(&VisionRx.Yaw, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&VisionRx.Pitch, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&VisionRx.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1);
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1);
//    VisionRx.Ready_flag = 1; //���ݾ���
//}
///**
//  * @brief  �������
//  */
//void CMD_SHOOT_Analysis()
//{
//    VisionRx.Function_word = CMD_SHOOT;
//    VisionRx.Shoot_flag = 1;                                             //���ָ�����
//    memcpy(&VisionRx.Shoot_speed, Vision_Rxbuffer + Array_index + 2, 4); //����ٶ�
//    memcpy(&VisionRx.Shoot_freq, Vision_Rxbuffer + Array_index + 6, 1);  //���Ƶ��
//    memcpy(&VisionRx.Shoot_mode, Vision_Rxbuffer + Array_index + 7, 1);  //���ģʽ
//}
///**
//  * @brief  �����˶�����
//  */
//void CMD_CHASSIS_CONTROL_Analysis()
//{
//    VisionRx.Function_word = CMD_CHASSIS_CONTROL;
//    memcpy(&VisionRx.Vx, Vision_Rxbuffer + Array_index + 2, 4); //�����ٶȽ���
//    memcpy(&VisionRx.Vy, Vision_Rxbuffer + Array_index + 6, 4);
//}
///**
//  * @brief  ����·�̿���
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
//    int16_t _check_sum = 0;         //��У���ñ���
//    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
//    Vision_Txbuffer[Frame_header] = FRAME_HEADER_DATA;
//    Vision_Txbuffer[Frame_end] = FRAME_END_DATA;
//    Vision_Txbuffer[Function_word] = _Functionword;

//    switch (_Functionword)
//    {
//    case CMD_GET_MCU_STATE:
//        //���Ӿ���������
//#ifndef DEBUG
//        memcpy(Vision_Txbuffer + 2, &VisionTx.Cloud_mode, 1); //����ģʽ
//        memcpy(Vision_Txbuffer + 3, &VisionTx.Pitch, 4);      //Pitch������
//        memcpy(Vision_Txbuffer + 7, &VisionTx.Yaw, 4);        //Yaw������
//#else
//        memcpy(Vision_Txbuffer + 2, &m, 1);  //����ģʽ
//        memcpy(Vision_Txbuffer + 3, &f1, 4); //Pitch������
//        memcpy(Vision_Txbuffer + 7, &f2, 4); //Yaw������
//#endif
//        memcpy(Vision_Txbuffer + 11, &VisionTx.Shoot_speed, 4); //����
//        memcpy(Vision_Txbuffer + 15, &VisionTx.Shoot_freq, 1);  //��Ƶ
//        break;
//    case ROBOT_ERR:
//        memcpy(Vision_Txbuffer + 2, &VisionTx.Error_code, 1); //��־ϵͳ�������
//        if (VisionTx.Error_code == MOTOR_OFFLINE_CNT)         //0x04��ʱ��Ҫ���ⷢ���������б�����
//        {
//            memcpy(Vision_Txbuffer + 3, &VisionTx.CAN1_motorlist, 2); //CAN1�������
//            memcpy(Vision_Txbuffer + 5, &VisionTx.CAN2_motorlist, 2); //CAN2�������
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
