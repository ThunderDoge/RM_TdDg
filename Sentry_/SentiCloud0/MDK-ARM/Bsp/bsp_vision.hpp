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

#ifndef __BSP_VISION_HPP
#define __BSP_VISION_HPP

#include "stm32f4xx.h"
#include <string.h>

//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_VISION_UART huart6
//�Ӿ����ڽ��ջ���������С������Ҫ���������޸�,ʹ�ÿ����ж�Ҫ�����������Ҫ����18
#define BSP_VISION_BUFFER_SIZE 60

//����֡�ؼ�λ��
enum __bsp_vision_Keywords
{
    Frame_header = 0,  //֡ͷ
    Function_word = 1, //������
    Sum_check = 16,    //��У��
    Frame_end = 17     //֡β
};

//����֡������
enum __bsp_vision_Functionwords
{
    //֡ͷ֡β
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
    //�Ӿ�������ص�
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01, //������̨��ԽǶ�
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02, //������̨���ԽǶ�
    CMD_SHOOT = 0x03,                   //���ָ��
    CMD_CHASSIS_CONTROL = 0X04,         //���̿���
	CMD_CHASSIS_LOACTION_CONTROL = 0X05,	//���̿���·��
	CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06,	//���̿���·�̴�����

    //��ط����Ӿ���
    CMD_GET_MCU_STATE = 0x11, //��ȡ��ؿ�����Ϣ
    ROBOT_ERR = 0X12
};

//ROBOT_ERR �Ĵ������б�
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

//�Ӿ��������ݽ����ṹ��
typedef struct __vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; //����֡������
    //��������
    float Vx; //����X���ٶ�
    float Vy; //����Y���ٶ�
	float Px; //����X��·��
	float Py; //����Y��·��
	float SpeedLimit;	//��������
    //��̨����
    float Yaw;          //Yaw��Ƕ�
    float Pitch;        //Pitch��Ƕ�
    uint8_t Cloud_mode; //��̨ģʽ
    //�������
    uint8_t Shoot_mode; //���ģʽ
    float Shoot_speed;  //����ٶ�
    uint8_t Shoot_freq; //���Ƶ��
    //���ݱ�־
    uint8_t Shoot_flag = 0; //�Ӿ������־λ
    uint8_t Ready_flag = 0; //���ݾ�����־λ

    //��־ϵͳʹ��
    uint8_t Error_code = 0;          //�������
    int16_t CAN1_motorlist = 0xffff; //CAN1����б�
    int16_t CAN2_motorlist = 0xffff; //CAN2����б�
} bsp_vision_data;
//�Ӿ���������֡��
struct VisionFrame
{
    VisionFrame(__bsp_vision_Functionwords FW)  //�����ֳ�ʼ��
    {
        Frame[Frame_header] = FRAME_HEADER_DATA;
        Frame[Function_word] = FW;
        Frame[Frame_end] = FRAME_END_DATA;
    };
    //�����������ݳ�ʼ��
    VisionFrame(__bsp_vision_Functionwords FW, uint8_t *ptrData) : VisionFrame(FW)
    {
        memcpy(Frame + 2, ptrData, 14);
        pack();
    };
    void pack(void);    //��װ�ú���
    int8_t load(uint8_t *ptrData, size_t size);   //װ������
    uint8_t Frame[18];  //����֡����

private:
    uint8_t load_iter = 0;  //����ָ��
};

extern bsp_vision_data bsp_vision_Send_Data, bsp_vision_Rec_Data; //���ڷ��ͣ���������

void bsp_vision_Init(void);                                   //�Ӿ����ڳ�ʼ��
void bsp_vision_It(void);                                     //�Ӿ������жϴ���
HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword); //�Ӿ����ڷ��ͺ���
HAL_StatusTypeDef bsp_vision_SendData(VisionFrame& frame);  //�Ӿ����ڷ��ͺ�����ʹ������֡��Ļ�
#endif
