/**
 * @file      app_sentry_check_device.h
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#ifndef __APP_SENTRY)CHECK_DEVICE_H_

//�������ļ�
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

//ʹ���豸�ĺ궨��
// #define APP_ALARM_USE_OLED   //ʹ��OLED��Ļ
// #define APP_ALARM_USE_BEEP   //ʹ�÷�����
// #define APP_ALARM_USE_LED    //ʹ��LED��



/**
 * @brief �豸����ID(�澯ID) �ṹ��
 */
typedef enum:uint8_t
{
    UpCloudConnect  =   0,
    DownCloudConnectDevice,
    ChassisConnectDevice,

    UpCloudLeftFricDevice,
    UpCloudRightFricDevice,
    UpCloudYawMotorDevice,
    UpCloudPitchMotorDevice,
    UpCloudFeedMotorDevice,

    DownCloudLeftFricDevice,
    DownCloudRightFricDevice,
    DownCloudYawMotorDevice,
    DownCloudPitchMotorDevice,
    DownCloudFeedMotorDevice,

    ChassisDriveMotorDevice,
    ChassisPillarDetectDevice,
    DbusDevice,

	CheckDeviceID_EnumLength,  //��һ����������ָʾID����
}CheckDeviceID_Enum;



/**
 * @brief �豸���󱨾����ȼ�
 */
typedef enum 
{
  PriorityLow             = -2,          ///< priority: low
  PriorityBelowNormal     = -1,          ///< priority: below normal
  PriorityNormal          =  0,          ///< priority: normal (default)
  PriorityAboveNormal     = +1,          ///< priority: above normal
  PriorityHigh            = +2,          ///< priority: high
  PriorityHighest         = +3,          ///< priority: highest
}AlarmPriority_Enum;



/**
 * @brief �豸���� �ṹ��
 */
typedef __packed struct cdt
{
	CheckDeviceID_Enum      id;						/* ����ID,Ϊ��ֹID��ͻ������������ID����CheckDeviceID_Enum������ */
	uint32_t                lastTick;            /* �ϴ�����ʱ�� */
	uint16_t                maxAllowTime;	        /* �������ʱ�� */
	AlarmPriority_Enum      priority;		        /* ���ȼ� */
	uint8_t                 isOffline : 1 ;           /* ��������״̬*/
    uint8_t(is_offline_func)(void);                 //����״̬��麯��
}CheckDevice_Type;



/**
 * @brief �豸�������飬
 */
typedef struct  
{
	CheckDevice_Type* deviceArry[CheckDeviceID_EnumLength]; /* �豸���� */
	int8_t checkDeviceNum;				       /* �����豸������������ʱ.��ʼ��ʱȷ���� */
}CheckDeviceArry_Type;

///��ʼ���豸���߼������
void CheckDevice_Init(void);

///��ʼ���豸�ṹ��
void CheckDevice_Type_Init(CheckDevice_Type* t);

///��һ���豸�������� �����豸���������
HAL_StatusTypeDef CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

///ͨ�����߼��������롣��������Ҫ��ʱ����ӵ��Լ���Taskʵ��֮�С���ע����Ҫ�ĺ궨�塣
void CheckDevice_TaskHandler(void);

#endif // !__APP_SENTRY)CHECK_DEVICE_H_