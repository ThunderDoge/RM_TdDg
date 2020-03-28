/**
 * @file: app_vision.hpp 
* @brief    �Ӿ�����Э���ļ�
* @details  32��С������ͨѶ����
* @author   Evan-GH & ThunderDoge
* @date      2019.12.8
* @version  v2.1.1
* @par Copyright (c):  RM2020���
* @par ��־
    2019/12/8   ��Evan-GH���˴��ڰ汾1.2�ϲ�����������⴮��v2.1.0
    2019/12/8   ThunderDoge�����Լ���ϲ�ý����ڿ����Ϊv2.1.1   �����˲��ֹ��ܡ��޸���һЩд��
    2020/2/20   ThunderDoge���ڱ�ר�õĴ���Э�����д���˸��ļ������Ҹ���Ϊapp_vision.hpp��
*/
//���Ӿ���ͨ��Э��μ���RM2020�����Ӿ�Э�� v2.0�� by Evan-GH

#ifndef __BSP_VISION_HPP
#define __BSP_VISION_HPP

#include "stm32f4xx.h"
#include <string.h>
#include "usart.h"
#include "SentryCloud.hpp"	//��Ϊ��Ҫʹ��_cloud_ctrl_mode���԰����ˡ�û�б������
#include "sentry_ctrl_def.hpp"
#include "sentry_can_commom.hpp"



//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_VISION_UART huart1
//�Ӿ����ڽ��ջ���������С������Ҫ���������޸�,ʹ�ÿ����ж�Ҫ�����������Ҫ����18
#define BSP_VISION_BUFFER_SIZE 120

///����֡�ؼ�λ��
enum __bsp_vision_Keywords
{
    Frame_header = 0,  //֡ͷ
    Function_word = 1, //������
    Sum_check = 16,    //��У��
    Frame_end = 17     //֡β
};

///����֡������
enum __bsp_vision_Commom_Data
{
    //֡ͷ֡β
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
};

///�ڱ��Ӿ���Ϣ������
enum __bsp_vision_Functionwords
{
    //�Ӿ�������ص�
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //������̨��ԽǶ�
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //������̨���ԽǶ�
    CMD_SHOOT = 0x03,                        //���ָ��
    CMD_CHASSIS_CONTROL = 0X04,              //���̿���
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //���̿���·��
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //���̿���·�̴�����
	CMD_GIMBAL_SPEED_CONTROL = 0x10,		 //���Ƶ���ת���ٶ�

    //��ط����Ӿ���
    CMD_GET_MCU_STATE = 0x11, //��ȡ��ؿ�����Ϣ
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,
};

///�Ӿ��������ݽ����ṹ��
typedef struct __vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; ///<����֡������
    ///��������
    float Vx;         ///<����X���ٶ�
    float Vy;         ///<����Y���ٶ�
    float Px;         ///<����X��·��
    float Py;         ///<����Y��·��
    float SpeedLimit; ///<��������
    uint8_t pillar_flag;
    uint8_t chassis_mode;
    ///��̨����
    float Yaw;          ///<Yaw��Ƕ�
	float YawSoft;
    float Pitch;        ///<Pitch��Ƕ�
    uint8_t Cloud_mode; ///<��̨ģʽ
    uint8_t cloud_ctrl_mode;
    ///�������
    uint8_t Shoot_mode; ///<���ģʽ
    float Shoot_speed;  ///<����ٶ�
    uint8_t Shoot_freq; ///<���Ƶ��
    ///���ݱ�־
    uint32_t UpdateTime;
    ///��־ϵͳʹ��
    uint8_t Error_code = 0;          ///<�������
    int16_t CAN1_motorlist = 0xffff; ///<CAN1����б�
    int16_t CAN2_motorlist = 0xffff; ///<CAN2����б�
	//���ṹ����Ϣ
	uint8_t Ready_flag;	//������־����ʾ��������δ����
} sentry_vision_data;

///��������б�
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


extern sentry_vision_data VisionTx, VisionRx; ///���ڷ���/���ջ���ṹ��
extern uint8_t Vision_Txbuffer[18];         ///���ڷ����ݴ�����

void bsp_vision_Init(void);                                   ///�Ӿ����ڳ�ʼ��
void bsp_vision_It(void);                                     ///�Ӿ������жϴ���
// HAL_StatusTypeDef bsp_vision_SendData(uint8_t _Functionword); ///�Ӿ����ڷ��ͺ���
HAL_StatusTypeDef bsp_vision_SendTxbuffer(uint8_t _Functionword);
void bsp_vision_load_to_txbuffer(uint8_t u8data, int loaction_at_buffdata); ///������װ�뻺��Vision_Txbuffer��
void bsp_vision_load_to_txbuffer(float fdata, int loaction_at_buffdata);    ///������װ�뻺��Vision_Txbuffer��

void CloudVisionTxRoutine(void);  ///���߼��ص���������С��������һ��VisionTx��ȫ����Ϣ��

#endif
