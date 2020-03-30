/**
 * @file      app_sentry_check_device.hpp
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.2
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

//DEBUG�ú궨��
#define __APP_CHECK_DEVICE_DEBUG

//----------------------- ��ؽṹ��/ö���Ͷ���------------------------

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

typedef enum :uint8_t
{
    REPORT_NON,
    REPORT_NEEDED,
    REPORT_IN_QUEUE,
}OfflineReportStates_Enum;


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
		CheckDeviceID_Enum id,
        uint16_t allow_time,
		uint8_t(*ptr_is_offline_func)(void) = NULL,
		AlarmPriority_Enum      pri = PriorityNormal,
		void (*ptr_state_changed_callback_func)(CheckDevice_Type*self) = Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask,
		void (*ptr_update_hook_func)(CheckDevice_Type*self) = Default_CheckDevice_UpdateHookFunc
		);
	CheckDeviceID_Enum      id = CheckDeviceID_EnumLength;						/** ����ID,Ϊ��ֹID��ͻ������������ID���� @see CheckDeviceID_Enum ������ */
	uint32_t                lastTick = 0;            /* �ϴ�����ʱ�� */
	uint16_t                maxAllowTime = 100;	        /* �������ʱ�� */
	AlarmPriority_Enum      priority = PriorityNormal;		        /* ���ȼ� */
	uint8_t			alarm_enabled = 1;		// ���߱�����ʹ��
	uint8_t                 is_offline = 0 ;           /* ��������״̬*/
    uint8_t                 is_change_reported = REPORT_NON;       // �Ƿ��ѷ��͸澯��Ϣ
    uint8_t(*is_offline_func)(void) = NULL;                //����״̬��麯��
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


//----------------------------- ���ȫ�ֱ���-------------------------------


extern QueueHandle_t QueueOfflineDevice;     /// �������豸�б� ���͸�ͨ�ź�������
extern CheckDeviceArry_Type CheckDeviceArry;			/// ���б����豸�б�
extern uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ];



//----------------------------- ����ӿں���-------------------------------

/// ��ʼ���豸���߼������
void app_sentry_CheckDevice_Init(void);

/// ��һ���豸�������� �����豸���������
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd);

/// ˢ�´����б�ͨ�����½����е��豸���͵� QueueOfflineDevice ��ʵ��
void app_sentry_CheckDevice_CheckAllDevice(void);

/// ���߼�⹦��ִ��
void app_sentry_CheckDevice_Handle(void);

/// �ⲿ:�Ӷ��л�ȡ�����豸
uint8_t app_sentry_CheckDevice_GetOfflineDeviceFromQueueTo(uint8_t* device_id, uint8_t* device_isoffline);

/// ����CAN�յ��������豸
void app_sentry_CheckDevice_CanRxCallback(uint8_t *ptrData);


#endif // !__APP_SENTRY_CHECK_DEVICE_H_


