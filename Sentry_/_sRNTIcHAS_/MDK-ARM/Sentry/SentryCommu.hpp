/**
  * @file  SentryCommu.cpp
  * @brief    �ڱ��Ӿ���CANͨѶ��ʶ��/��������
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/18
  * @version  v1.0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __SENTRY_COMMU_HPP_
#define __SENTRY_COMMU_HPP_

//����궨�壬�����CAN�����޸Ĵ˴��궨��
#define CAN_INTERBOARD hcan2

#include "bsp_vision.hpp"
#include "bsp_can.hpp"
#include "SentryCanCommu.hpp"
enum SENTRY_CAN_ID : uint32_t //���ͨѶID��
{
    UP_CLOUD_STATES = 0X101U,
    DOWN_CLOUD_STATES = 0X102U,
    CHASSIS_STATES = 0X103U,
    CHASSIS_PILLAR = 0X104U,
    SUPERIOR_UP_RELATIVE_CMD = 0X111U,
    SUPERIOR_UP_ABSOLUTE_CMD = 0X112U,
    SUPERIOR_DOWN_RELATIVE_CMD = 0X121U,
    SUPERIOR_DOWN_ABSOLUTE_CMD = 0X122U,
    UP_FEED = 0X113U,
    DOWN_FEED = 0X123U,
    SUPERIOR_CHASSIS_MOVE = 0X130U,
    SUPERIOR_CHASSIS_SET_LOACTION = 0X131U,
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED = 0X132U,
    SUPERIOR_SAFE = 0x1A0U,
};
enum SuperiorControlFlag_for_Chassis
{
    _SUPERIOR_CHASSIS_SPEED_SET_ = 0X01,
    _SUPERIOR_CHASSIS_LOACATION_SET_ = 0X02,
    _SUPERIOR_CHASSIS_LOACATION_SET_SPEED_LIMIT_ = 0X03,
    _SUPERIOR_OFFLINE_ = 0,
};

//�Ӿ���������֡������
enum __bsp_vision_Functionwords
{
    //֡ͷ֡β
    FRAME_HEADER_DATA = 0xff,
    FRAME_END_DATA = 0x0d,
    //�Ӿ�������ص�
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,      //������̨��ԽǶ�
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,      //������̨���ԽǶ�
    CMD_SHOOT = 0x03,                        //���ָ��
    CMD_CHASSIS_CONTROL = 0X04,              //���̿���
    CMD_CHASSIS_LOACTION_CONTROL = 0X05,     //���̿���·��
    CMD_CHASSIS_LOCATION_LIMIT_SPEED = 0X06, //���̿���·�̴�����

    //��ط����Ӿ���
    CMD_GET_MCU_STATE = 0x11, //��ȡ��ؿ�����Ϣ
    ROBOT_ERR = 0X12,
    STA_CHASSIS = 0X13,
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

struct CanCommuRecv_t
{
    uint32_t RecvId;
    uint8_t SuperiorControlFlags;
    float UpCloudPitchYaw[2];
    float DownCloudPitchYaw[2];
    float SuperCon_Relative_PitchYaw[2];
    float SuperCon_Absolute_PitchYaw[2];
    float Chassis_SpeedLocation[2];
    float Chassis_SpeedLimit;
    uint32_t RecvUpdateTime;
    //������Ϣ
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    float Pillar_Dist;
    uint8_t Pillar_flag;
};

//�Ӿ��������ݽ����ṹ��
struct bsp_vision_data
{
    uint8_t Frame_header = FRAME_HEADER_DATA;
    uint8_t Frame_end = FRAME_END_DATA;
    uint8_t Function_word; //����֡������
    //��������
    float Vx;         //����X���ٶ�
    float Vy;         //����Y���ٶ�
    float Px;         //����X��·��
    float Py;         //����Y��·��
    float SpeedLimit; //��������
    uint8_t pillar_flag;
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
};

extern CanCommuRecv_t CanInfo;
extern bsp_vision_data VisionInfo;

#endif // __SENTRY_COMMU_HPP_
