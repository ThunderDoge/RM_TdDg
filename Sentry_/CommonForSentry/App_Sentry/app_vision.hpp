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
    2019/12/8   ThunderDoge�����Լ���ϲ�ý����ڿ����Ϊv2.1.1
�����˲��ֹ��ܡ��޸���һЩд�� 2020/2/20
ThunderDoge���ڱ�ר�õĴ���Э�����д���˸��ļ������Ҹ���Ϊapp_vision.hpp��
*/
//���Ӿ���ͨ��Э��μ���RM2020�����Ӿ�Э�� v2.0�� by Evan-GH

#ifndef __BSP_VISION_HPP
#define __BSP_VISION_HPP

#include "sentry_ctrl_def.hpp" //��Ϊ��Ҫʹ�� _cloud_ctrl_mode ���԰����ˡ�
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>

//#include "sentry_can_commom.hpp"

//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_VISION_UART huart1
//�Ӿ����ڽ��ջ���������С������Ҫ���������޸�,ʹ�ÿ����ж�Ҫ�����������Ҫ����18
#define BSP_VISION_BUFFER_SIZE 120

///����֡�ؼ�λ��
enum __app_vision_Keywords
{
    Frame_header = 0,  //֡ͷ
    Function_word = 1, //������
    Sum_check = 16,    //��У��
    Frame_end = 17     //֡β
};

///����֡������
enum __app_vision_Commom_Data
{
    //֡ͷ֡β
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

///�ڱ��Ӿ���Ϣ������
enum __app_vision_Functionwords
{
    //�Ӿ�������ص�
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //������̨��ԽǶ�
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //������̨���ԽǶ�
    CMD_SHOOT = 0x03,                        //���ָ��
    CMD_CHASSIS_CONTROL = 0X04,              //���̿���
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //���̿���·��
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //���̿���·�̴�����
    CMD_READ_PID = 0X07,
    CMD_WRITE_PID = 0X08,
    CMD_GIMBAL_SPEED_CONTROL = 0x10, //���Ƶ���ת���ٶ�

    //��ط����Ӿ���
    CMD_GET_MCU_STATE = 0x11, //��ȡ��ؿ�����Ϣ
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,

    JUD_GAME_STATUS = 0x20,
    JUD_ENY_HP = 0x21,
    JUD_GAME_EVENT = 0x22,
    JUD_SELF_HP = 0x23,
    JUD_GUN_CHASSIS_HEAT = 0x24,
    JUD_SELF_BUFF = 0x25,
    JUD_TAKING_DMG = 0x26,
    JUD_SHOOTING = 0x27,
    JUD_AMMO_LEFT = 0x28,
    JUD_USER = 0x29,
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
    float Yaw; ///< Yaw��Ƕ�
    float YawSoft;
    float Pitch;        ///< Pitch��Ƕ�
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
    int16_t CAN1_motorlist = 0xffff; ///< CAN1����б�
    int16_t CAN2_motorlist = 0xffff; ///< CAN2����б�
    //���ṹ����Ϣ
    uint8_t Ready_flag; //������־����ʾ��������δ����
} sentry_vision_data;

///��������б�
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


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>��Ҫ������������Ҫ�û��Լ�ʵ�ֵĺ���<<<<<<<<<<<<<<<<<<<<<*/
__weak HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d);


extern sentry_vision_data VisionTx, VisionRx; ///���ڷ���/���ջ���ṹ��
extern uint8_t Vision_Txbuffer[18];           ///���ڷ����ݴ�����
extern uint8_t Vision_Rxbuffer[BSP_VISION_BUFFER_SIZE];

void app_vision_Init(void); ///�Ӿ����ڳ�ʼ��
void app_vision_It(void);   ///�Ӿ������жϴ���

void app_vision_another_Init(void);
void app_vision_dma_cpltcallback(void); ///
void app_vision_dma_abort_in_idle(void);
uint8_t app_vision_analysis_intgrated(void);

// HAL_StatusTypeDef app_vision_SendData(uint8_t _Functionword);

///�Ӿ����ڷ��ͺ���
HAL_StatusTypeDef app_vision_SendTxbuffer(uint8_t _Functionword);
void app_vision_load_to_txbuffer(
    uint8_t u8data,
    int loaction_at_buffdata); ///������װ�뻺��Vision_Txbuffer��

void app_vision_load_to_txbuffer(
    float fdata, int loaction_at_buffdata); ///������װ�뻺��Vision_Txbuffer��

// ���������ֵ����ķ��ͺ���
void CMD_GET_MCU_STATE_Tx(float pitch, float, float yaw_soft,
                          uint8_t cloud_mode, uint8_t shoot_mode);
void ROBOT_ERR_Tx(uint8_t err_code);
void STA_CHASSIS_Tx(uint8_t chassis_mode, uint8_t pillar_flag, float velocity,
                    float position);

void JUD_GAME_STATUS_Tx(uint8_t game_progress, uint16_t stage_remain_time);
void JUD_ENY_HP_Tx(uint16_t hp);
void JUD_GAME_EVENT_Tx(); // ����
void JUD_SELF_HP_Tx(uint16_t hp);
void JUD_GUN_CHASSIS_HEAT_Tx(float chassis_power, uint16_t cha_pwr_buf,
                             uint16_t gun_heat);
void JUD_SELF_BUFF_Tx(uint8_t buff_code);
void JUD_TAKING_DMG_Tx(uint8_t armor_id_enum, uint8_t hurt_type_enum);
void JUD_SHOOTING_Tx(uint8_t bullet_freq, float bullet_speed);
void JUD_AMMO_LEFT_Tx(uint16_t bulelt_left);
void JUD_USER_Tx();

uint8_t app_vision_Analysis(void);
extern int not_analysed_index;
#endif
