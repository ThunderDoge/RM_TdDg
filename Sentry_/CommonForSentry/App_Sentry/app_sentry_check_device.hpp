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
#ifndef __APP_SENTRY_CHECK_DEVICE_H_
#define __APP_SENTRY_CHECK_DEVICE_H_

//�������ļ�
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "sentry_can_commom.hpp"

//DEBUG�ú궨��
#define __APP_CHECK_DEVICE_DEBUG

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


//���Ľ�ʹ�õĶ���
struct CheckDevice_Type;
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* self);
void Default_CheckDevice_UpdateHookFunc(CheckDevice_Type* self);


/**
 * @brief �豸���� �ṹ��
 */
struct CheckDevice_Type
{
	CheckDevice_Type(															///< ���캯��
		CheckDeviceID_Enum  id,
        uint16_t            allow_time,
		uint8_t             (*ptr_is_offline_func)(void)                                = NULL,
		AlarmPriority_Enum  pri                                                         = PriorityNormal,
		void                (*ptr_state_changed_callback_func)(CheckDevice_Type*self)   = Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask,
		void                (*ptr_update_hook_func)(CheckDevice_Type*self)              = Default_CheckDevice_UpdateHookFunc
		);
	CheckDeviceID_Enum      id              = CheckDeviceID_EnumLength;						/** ����ID,Ϊ��ֹID��ͻ������������ID���� @see CheckDeviceID_Enum ������ */
	uint32_t                lastTick        = 0;            /* �ϴ�����ʱ�� */
	uint16_t                maxAllowTime    = 100;	        /* �������ʱ�� */
	AlarmPriority_Enum      priority        = PriorityNormal;		        /* ���ȼ� */
	uint8_t			        alarm_enabled   = 1;		// ���߱�����ʹ��
	// uint8_t                 is_offline      = 0 ;           /* ��������״̬*/
    uint8_t                 is_change_reported = 0;       // �Ƿ��ѷ��͸澯��Ϣ
    uint8_t(*is_offline_func)(void)         = NULL;                //����״̬��麯��
	void (*state_changed_callback_func)(CheckDevice_Type* self);	// ���߻ص���������⵽����֮����ô˺�����
	void (*update_hook_func)(CheckDevice_Type* self);		// ״̬���¹��Ӻ��������豸����ʱ���á�
};


/**
 * @brief �豸�������飬
 */
typedef struct  
{
	CheckDevice_Type* deviceArry[CheckDeviceID_EnumLength]; /* �豸���� */
	int8_t checkDeviceNum;				       /* �����豸������������ʱ.��ʼ��ʱȷ���� */
}CheckDeviceArry_Type;

//-----------------------\end ��ؽṹ��/ö���Ͷ���------------------------

//-----------------------------\begin ���ȫ�ֱ���-------------------------------


extern QueueHandle_t QueueOfflineDeviceToCommuTask;     /// �������豸�б� ���͸�ͨ�ź�������
extern CheckDeviceArry_Type CheckDeviceArry;			/// �����������豸���б�
extern uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ];    /// �ڱ������豸������״̬


//-----------------------------\end ���ȫ�ֱ���-------------------------------

//----------------------------- ����ӿں���-------------------------------

///��ʼ���豸���߼������
void app_sentry_CheckDevice_Init(void);


/////��ʼ���豸�ṹ��
//void app_sentry_CheckDevice_Type_Init(CheckDevice_Type* device);


/////��ʼ���豸���߼�����ܣ��������豸���������
//void app_sentry_CheckDevice_Type_Init_AddToArray(	CheckDevice_Type* 	device,
//										CheckDeviceID_Enum 	id,
//										uint16_t 			max_allow_time,
//										AlarmPriority_Enum 	priority,
//										FunctionalState		enable_alarm,
//										uint8_t (*ptr_is_offline_func)(void),
//										void (*ptr_update_hook_func)(CheckDevice_Type*self) );



///��һ���豸�������� �����豸���������
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

///ˢ�´����б�ͨ�����½����е��豸���͵� QueueOfflineDeviceToCommuTask ��ʵ��
void app_sentry_CheckDevice_CheckAllDevice(void);

///ͨ�����߼��������롣�������߼�Task���á���ע����Ҫ�ĺ궨�塣
void app_sentry_CheckDevice_TaskHandler(void);

///����ͨ�ź������õ� �����豸���������������� @see QueueOfflineDeviceToCommuTask
void app_sentry_CheckDevice_CommuTaskCallback(void);

///CAN ���պ���. ��CAN���ջص������е���
void app_sentry_CheckDevice_CanRxCallBack(uint8_t* pdata);

#endif // !__APP_SENTRY_CHECK_DEVICE_H_


