/** 
* @brief    �Ӿ�����Э���ļ�
* @details  32��С������ͨѶ����
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020���
* @par ��־
    2019/12/8   ��Evan-GH���˴��ڰ汾1.2�ϲ�����������⴮��v2.1.0
    2019/12/8   ThunderDoge�����Լ���ϲ�ý����ڿ����Ϊv2.1.1   �����˲��ֹ��ܡ��޸���һЩд��
*/
//���Ӿ���ͨ��Э��μ���RM2020�����Ӿ�Э�� v2.0�� by Evan-GH
#include "bsp_vision.hpp"
//#define DEBUG
#ifdef DEBUG
		uint8_t m = 1;
		float f1 = 12.5f , f2 = 25.2;
#endif

bsp_vision_data bsp_vision_Rec_Data, bsp_vision_Send_Data;    //�Ӿ����ڽ�����������,�Ӿ����ڷ��͵�����
uint8_t Vision_Rxbuffer[BSP_VISION_BUFFER_SIZE] = {0}; //���ڽ������ݻ������飬���ڻ�������������������֡������
static int8_t Array_index = 0;                                //���������ݼ����ָ��
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
//    bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Pitch, Vision_Rxbuffer + Array_index + 2, 4); //��̨�ǶȽ���
//    memcpy(&bsp_vision_Rec_Data.Yaw, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&bsp_vision_Rec_Data.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1); //��̨ģʽ����
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1); //���ģʽ
//    bsp_vision_Rec_Data.Ready_flag = 1;                                             //���ݾ���
//}
///**
//  * @brief  ��̨���ԽǶȿ���
//  */
//void CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Yaw, Vision_Rxbuffer + Array_index + 2, 4);
//    memcpy(&bsp_vision_Rec_Data.Pitch, Vision_Rxbuffer + Array_index + 6, 4);
//    memcpy(&bsp_vision_Rec_Data.Cloud_mode, Vision_Rxbuffer + Array_index + 10, 1);
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 11, 1);
//    bsp_vision_Rec_Data.Ready_flag = 1; //���ݾ���
//}
///**
//  * @brief  �������
//  */
//void CMD_SHOOT_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_SHOOT;
//    bsp_vision_Rec_Data.Shoot_flag = 1;                                             //���ָ�����
//    memcpy(&bsp_vision_Rec_Data.Shoot_speed, Vision_Rxbuffer + Array_index + 2, 4); //����ٶ�
//    memcpy(&bsp_vision_Rec_Data.Shoot_freq, Vision_Rxbuffer + Array_index + 6, 1);  //���Ƶ��
//    memcpy(&bsp_vision_Rec_Data.Shoot_mode, Vision_Rxbuffer + Array_index + 7, 1);  //���ģʽ
//}
///**
//  * @brief  �����˶�����
//  */
//void CMD_CHASSIS_CONTROL_Analysis()
//{
//    bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//    memcpy(&bsp_vision_Rec_Data.Vx, Vision_Rxbuffer + Array_index + 2, 4); //�����ٶȽ���
//    memcpy(&bsp_vision_Rec_Data.Vy, Vision_Rxbuffer + Array_index + 6, 4);
//}
///**
//  * @brief  ����·�̿���
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
* @brief  �Ӿ����ڽ�������
* @details  �Ի���������ݽ���һ�α�������������
* @param  NULL
* @retval  uint8_t 1��û�н������������������ 0�����Ѿ����
*/
static uint8_t bsp_vision_Analysis(void)
{
    int16_t _check_sum = 0;                         //��У���ñ���
    if (Array_index <= BSP_VISION_BUFFER_SIZE - 18) //�������ڿ�ʼ���
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
            if (_check_sum != Vision_Rxbuffer[Array_index + Sum_check])
            {
                Array_index += 1; //��У����ˣ�ָ���ƶ�1λ�������¼��
                return 1;         //У��ͳ���ֱ���˳�,�����ڻ������������֡
            }

            //֡ͷ֡β��ȷ����У����ȷ����ʼ����
            bsp_vision_Rec_Data.Ready_flag = 1; //������ݾ���
//            switch (Vision_Rxbuffer[Array_index + Function_word])
//            {
//            case CMD_GIMBAL_RELATIVE_CONTROL: //������̨��ԽǶȣ����ݽ���
//                bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_RELATIVE_CONTROL;
//                CMD_GIMBAL_RELATIVE_CONTROL_Analysis();
//                break;
//            case CMD_GIMBAL_ABSOLUTE_CONTROL: //������̨���ԽǶȣ����ݽ���
//                bsp_vision_Rec_Data.Function_word = CMD_GIMBAL_ABSOLUTE_CONTROL;
//                CMD_GIMBAL_ABSOLUTE_CONTROL_Analysis();
//                break;
//            case CMD_SHOOT: //���ָ��
//                bsp_vision_Rec_Data.Function_word = CMD_SHOOT;
//                CMD_SHOOT_Analysis();
//                break;
//            case CMD_CHASSIS_CONTROL: //���̿���
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//                CMD_CHASSIS_CONTROL_Analysis();
//                break;
//            case CMD_CHASSIS_LOACTION_CONTROL: //����·�̿���
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_CONTROL;
//                CMD_CHASSIS_LOACTION_CONTROL_Analysis();
//                break;
//            case CMD_CHASSIS_LOCATION_LIMIT_SPEED: //���̿���·�̴�����
//                bsp_vision_Rec_Data.Function_word = CMD_CHASSIS_LOCATION_LIMIT_SPEED;
//                CMD_CHASSIS_LOCATION_LIMIT_SPEED_Analysis();
//                break;
//            default:
//                bsp_vision_Rec_Data.Ready_flag = 0; //û��������ȡ��������ݾ���
//                break;
//            }
			SentryVisionUartRxAll(Vision_Rxbuffer+Array_index);
            //���е�����ͱ�ʾ�����Ѿ��ɹ���һ֡�����Ѿ��걸
            Array_index += 18; //����֡һ֡����Ϊ18�������ƶ�18λ
            return 1;          //������һ֡���ƶ�18λ�������
        }
        else
        {
            Array_index += 1; //֡ͷ֡β���ԣ��ƶ�һλ�������
            return 1;
        }
    }
    else //���µ��������Բ��ܹ���һ֡�ˣ���������
    {
        Array_index = 0; //��ָ�븴λ
        return 0;        //�Ѿ�����һ�黺��������������
    }
}

/**
* @brief  �Ӿ����ڳ�ʼ��
* @details  ���Ӿ���32ͨѶ�Ĵ��ڽ��г�ʼ��
* @param  NULL
* @retval  NULL
*/
void bsp_vision_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //��������ж�λ
    __HAL_UART_ENABLE_IT(&BSP_VISION_UART, UART_IT_IDLE);                                       //ʹ��DMA���տ����ж�
    HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //��ʼDMA����
}

/**
* @brief  �Ӿ������жϴ�����
* @details  ���ڶ��Ӿ����͹����������������������������һ�λ����
* @param  NULL
* @retval  NULL
*/
void bsp_vision_It(void)
{
    if (__HAL_UART_GET_FLAG(&BSP_VISION_UART, UART_FLAG_IDLE) != RESET) //��������˿����ж�
    {
        HAL_UART_DMAStop(&BSP_VISION_UART); //�ر�DMA
        while (bsp_vision_Analysis())
            ;                                                                                       //���ݽ���������һ�λ�����
        memset(Vision_Rxbuffer, 0, BSP_VISION_BUFFER_SIZE);                                         //������ɣ�������0
        __HAL_UART_CLEAR_IDLEFLAG(&BSP_VISION_UART);                                                //��������жϱ�־λ
        HAL_UART_Receive_DMA(&BSP_VISION_UART, (uint8_t *)Vision_Rxbuffer, BSP_VISION_BUFFER_SIZE); //���¿���DMA���մ���
    }
}

/**
* @brief  �Ӿ����ڷ��ͺ���
* @details  ������Ӿ����͵�����������־ϵͳ����Ҫʹ���߶Խṹ�������������
* @param  uint8_t _Functionword �������б�
* @retval  HAL_StatusTypeDef HAL_OK ���ͳɹ� HAL_ERROR����ʧ��
*/
uint8_t Vision_Txbuffer[18] = {0}; //����������
HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword)
{
    int16_t _check_sum = 0;         //��У���ñ���
    memset(Vision_Txbuffer, 0, 18); //����֮ǰ�����һ��
    Vision_Txbuffer[Frame_header] = FRAME_HEADER_DATA;
    Vision_Txbuffer[Frame_end] = FRAME_END_DATA;
    Vision_Txbuffer[Function_word] = _Functionword;

    switch (_Functionword)
    {
    case CMD_GET_MCU_STATE:         
	//���Ӿ���������
#ifndef DEBUG
		memcpy(Vision_Txbuffer + 2, &bsp_vision_Send_Data.Cloud_mode, 1);   //����ģʽ
        memcpy(Vision_Txbuffer + 3, &bsp_vision_Send_Data.Pitch, 4);        //Pitch������
        memcpy(Vision_Txbuffer + 7, &bsp_vision_Send_Data.Yaw, 4);          //Yaw������
#else
		memcpy(Vision_Txbuffer + 2, &m, 1);   //����ģʽ
        memcpy(Vision_Txbuffer + 3, &f1, 4);        //Pitch������
        memcpy(Vision_Txbuffer + 7, &f2, 4);          //Yaw������
#endif
        memcpy(Vision_Txbuffer + 11, &bsp_vision_Send_Data.Shoot_speed, 4); //����
        memcpy(Vision_Txbuffer + 15, &bsp_vision_Send_Data.Shoot_freq, 1);  //��Ƶ
        break;
    case ROBOT_ERR:
        memcpy(Vision_Txbuffer + 2, &bsp_vision_Send_Data.Error_code, 1); //��־ϵͳ�������
        if (bsp_vision_Send_Data.Error_code == MOTOR_OFFLINE_CNT)         //0x04��ʱ��Ҫ���ⷢ���������б�����
        {
            memcpy(Vision_Txbuffer + 3, &bsp_vision_Send_Data.CAN1_motorlist, 2); //CAN1�������
            memcpy(Vision_Txbuffer + 5, &bsp_vision_Send_Data.CAN2_motorlist, 2); //CAN2�������
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

