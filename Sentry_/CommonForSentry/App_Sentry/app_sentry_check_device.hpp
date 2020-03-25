/**
 * @file      app_sentry_check_device.hpp
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#ifndef __APP_SENTRY_CHECK_DEVICE_HPP_
#define __APP_SENTRY_CHECK_DEVICE_HPP_

//�������ļ�
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_stddef.h"
#include "cmsis_os.h"

//�����ú궨��
#define __APP_CHECK_DEVICE_DEBUG //DEBUG�ú궨�塣�ú궨��Ὺ��һЩ ��ѭ�� while(1) �Ա�¶����

#define __APP_CHECK_DEVICE_USE_OLED // �궨�壺�˰�����OLEDҺ����Ļ��������Ϣͨ��OLED��ʾ��
#define __APP_CHECK_DEVICE_USE_CAN  // �궨�壺������Ϣͨ��CAN���͡�

#ifdef __APP_CHECK_DEVICE_USE_OLED
#include "sentry_can_commom.hpp"
#endif // __APP_CHECK_DEVICE_USE_OLED

#ifdef __APP_CHECK_DEVICE_USE_CAN
#include "bsp_can.hpp"
#endif // __APP_CHECK_DEVICE_USE_CAN

//ʹ���豸�ĺ궨��
// #define APP_ALARM_USE_OLED   //ʹ��OLED��Ļ
// #define APP_ALARM_USE_BEEP   //ʹ�÷�����
// #define APP_ALARM_USE_LED    //ʹ��LED��

//-----------------------\begin ��ؽṹ��/ö���Ͷ���------------------------

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
	UpCloudImuDevice,

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
typedef enum :int8_t
{
  PriorityLow             = -2,          ///< priority: low
  PriorityBelowNormal     = -1,          ///< priority: below normal
  PriorityNormal          =  0,          ///< priority: normal (default)
  PriorityAboveNormal     = +1,          ///< priority: above normal
  PriorityHigh            = +2,          ///< priority: high
  PriorityHighest         = +3,          ///< priority: highest
}AlarmPriority_Enum;


//���Ľ�ʹ�õ�����

struct CheckDevice_Type;
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* self);
void Default_CheckDevice_UpdateHookFunc(CheckDevice_Type* self);


/**
 * @brief �豸���� �ṹ��
 */
struct CheckDevice_Type
{
	CheckDevice_Type(															///< ���캯��
		CheckDeviceID_Enum id,
        uint16_t allow_time,
		uint8_t(*ptr_is_offline_func)(void) = NULL,
		AlarmPriority_Enum      pri = PriorityNormal,
		void (*ptr_state_changed_callback_func)(CheckDevice_Type*self) = Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask,
		void (*ptr_update_hook_func)(CheckDevice_Type*self) = Default_CheckDevice_UpdateHookFunc
		);
	CheckDeviceID_Enum      id = CheckDeviceID_EnumLength;      ///< ����ID,�Է�ֹID��ͻ. ȡֵ�� @see CheckDeviceID_Enum  */
	uint32_t                lastTick = 0;                       ///< �ϴ�����ʱ�� */
	uint16_t                maxAllowTime = 100;	                ///< �����������ʱ�� */
	AlarmPriority_Enum      priority = PriorityNormal;          ///< ���ȼ� ȡֵ�� @see AlarmPriority_Enum */
	uint8_t			        alarm_enabled = 1;                  ///< ���߱�����ʹ��
	uint8_t                 is_offline = 0 ;                    ///< ��������״̬*/
    uint8_t                 is_change_reported = 0;             ///< �Ƿ��ѷ��͸澯��Ϣ
    uint8_t(*is_offline_func)(void) = NULL;                     ///< ����״̬��麯��
	void (*state_changed_callback_func)(CheckDevice_Type* self);///< ���߻ص���������⵽����֮����ô˺�����
	void (*update_hook_func)(CheckDevice_Type* self);		    ///< ״̬���¹��Ӻ��������豸����ʱ���á�
};


/**
 * @brief �豸�������� 
 * 
 */
typedef struct  
{
	CheckDevice_Type* deviceArry[CheckDeviceID_EnumLength]; /**< �豸���� */
	int8_t checkDeviceNum;				       /**< �����豸������������ʱ.��ʼ��ʱȷ���� */
}CheckDeviceArry_Type;

//-----------------------\end ��ؽṹ��/ö���Ͷ���------------------------

//-----------------------------\begin ���ȫ�ֱ���-------------------------------


extern QueueHandle_t QueueOfflineDeviceToCommuTask;     /// �������豸�б� ���͸�ͨ�ź�������
extern CheckDeviceArry_Type CheckDeviceArry;			/// �����豸�б�


//-----------------------------\end ���ȫ�ֱ���-------------------------------

//----------------------------- ����ӿں���-------------------------------


///��ʼ���豸���߼�����ܡ�������ʹ�� app_sentry_CheckDevice_AddToArray() ֮ǰ��ʼ��.
void app_sentry_CheckDevice_Init(void);

///��һ���豸�����ָ�� �����豸�б���
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

///ˢ�´����б�ͨ�����½����е��豸���͵� QueueOfflineDeviceToCommuTask ��ʵ��
void app_sentry_CheckDevice_CheckAllDevice(void);

///ͨ�����߼��������롣��ӵ����Լ���Task֮�С���ע����Ҫ�ĺ궨�塣
void app_sentry_CheckDevice_TaskHandler(void);

///���� ͨ������ ���õĴ����豸����������ӵ����Լ��� ͨ������ ֮�� �������� @see QueueOfflineDeviceToCommuTask
void app_sentry_CheckDevice_CommuTaskCallback(void);


#ifdef __APP_CHECK_DEVICE_USE_OLED
/// ��ʾ�����豸��Ϣ.�˺�����Ҫ�����ж���
void app_sentry_CheckDevice_OledDisplay(CheckDevice_Type* device);
#endif // __APP_CHECK_DEVICE_USE_OLED



#endif // !__APP_SENTRY_CHECK_DEVICE_HPP_


