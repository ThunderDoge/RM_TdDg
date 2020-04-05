/**
 * @file      app_sentry_check_device.cpp
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   ʹ�÷�����
 * 1.��Ҫ�������豸 �� CheckDeviceID_Enum �м����豸IDö����
 * 2.�����豸���� CheckDevice_Type
 * 3.��ʼ�������е��� app_sentry_CheckDevice_Init()
 * 4.���� app_sentry_CheckDevice_AddToArray() ���㶨����豸����ӵ������豸�б�
 * 5.���ڵ����� app_sentry_CheckDevice_Handle()
 * 6.��CAN�ص������е��� app_sentry_CheckDevice_CanRxCallback() �Ը����������ϵ��豸��״̬
 * 7.��Ҫ>>>>>>>>>>>>>>>>>>����Ҫ���ж��� ͨ��CAN���ͱ��������豸�ĺ���<<<<<<<<<<<<<<<<<<�� 
 *   ������ͨ�� app_sentry_CheckDevice_GetOfflineDeviceFromQueueTo() ��ȡ�����д������������豸
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.2
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "app_sentry_check_device.hpp"



//ȫ�ֱ���

static uint8_t checkdevice_is_inited=0;			//���Ѿ���ʼ������־����

CheckDeviceArry_Type CheckDeviceArry;			//�����豸�б�

QueueHandle_t QueueOfflineDevice;					/// �������豸�б� ���͸�ͨ�ź�������

uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ] = {1};


/**
 * @brief       Ĭ�ϵĸ��¹��Ӻ��������¹��Ӻ������ڳ�ʼ���� @see app_sentry_CheckDevice_Type_Init() ��Ϊ���
 * 
 * @param     self  ��Դ
 */
void Default_CheckDevice_UpdateHookFunc(CheckDevice_Type* self)
{
    self->lastTick = HAL_GetTick();
    if(app_sentry_CheckDevice_OfflineList[(uint8_t)self->id] == 1)
    {
        self->is_change_reported = REPORT_NEEDED;
    }
    app_sentry_CheckDevice_OfflineList[(uint8_t)self->id] =0;
}


/**
 * @brief   Ĭ�ϵ����߻ص���������ѱ��豸 device ���ӵ����� QueueOfflineDevice
 * ���ߺ����߶�Ҫ
 * @param     device  ָ���Լ���ָ�롣���߻ص��������ڳ�ʼ���� @see app_sentry_CheckDevice_Type_Init() ��Ϊ���
 */
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* device)
{

        xQueueSendToBack( 	(QueueHandle_t) 	QueueOfflineDevice,
                        (CheckDevice_Type*)	device , 
                        (TickType_t)		100 / portTICK_PERIOD_MS 	);
        device->is_change_reported = REPORT_IN_QUEUE;

}


/** 
  * @brief      ���������жϺ���
  * @param[in]  device ����ָ��
  * @retval     None
  * @author     LittleDragon
  * @par         ��־ 
  *             2020-3-12   v0.1    ThunderDoge�Ķ���LittleDragon��RM2019Ӣ�۴��룬������һ�κܺã��ʲ���
  */
static void DeviceOffline_Check(CheckDevice_Type* device)
{	
    if( device == NULL )    //��Ч���豸���ܿ�����Խ��
        while (1){;}        //��¶����

	uint8_t last_offline = app_sentry_CheckDevice_OfflineList[(uint8_t)device->id];

	if( device->is_offline_func != NULL )   //������������߼�⺯��
	{
		app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] = device->is_offline_func();    //��ֱ��ִ�к���
	}
	else                                        //���û��
	{
		uint32_t dt = HAL_GetTick() - device->lastTick;     //��������ʱ��
		app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] = (dt > device->maxAllowTime);    //д��״̬
	}
	
	if(app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] != last_offline)	// �Ǽǣ���Ҫ����
	{device->is_change_reported = REPORT_NEEDED;}
	
	if(device->is_change_reported == REPORT_NEEDED)
	{
		device->state_changed_callback_func(device);
		// device->is_change_reported = REPORT_IN_QUEUE;    //�� state_changed_callback_func �������״̬
	}
    
}



/**
 * @brief     ��һ���豸�������� ѹ���豸���ջ��
 * 
 * @param     id                ��������(����ID) @see CheckDeviceID_Enum
 * @param     onLineTickPtr     �豸������ʱ�����ָ�롣���û����и��¡�
 * @param     maxAllowTime      �����������ʱ��������������ʱ���������Ϊ����
 * @param     priority          �������ȼ�
 * @param     status            ״ָ̬�룬ָ������豸��״̬��־λ��ͨ��ָ������޸�. ����Ϊ��.
 * @param     cmd               ����ʹ��
 * @author     LittleDragon
 * @par         ��־ 
 *             2020-3-12   v0.1    ThunderDoge�Ķ���LittleDragon��RM2019Ӣ�۴��룬������һ�κܺã��ʲ���
 */


/**
 * @brief Construct a new CheckDevice_Type::CheckDevice_Type object
 * 
 * @param     id    �豸ID��  @see CheckDeviceID_Enum
 * @param     is_inter_brd              �Ƿ��ǰ����豸
 * @param     pri                       ���ȼ�����������Ĭ��
 * @param     ptr_is_offline_func       ���ߺ���
 * @param     ptr_state_changed_callback_func   ����/���߻ص�����
 * @param     ptr_update_hook_func              ���ݸ��¹��Ӻ���
 */
CheckDevice_Type::CheckDevice_Type(
	CheckDeviceID_Enum 				id,
    uint16_t                        allow_time,
	uint8_t(*ptr_is_offline_func)(void),
	AlarmPriority_Enum      		pri,
	void (*ptr_state_changed_callback_func)(CheckDevice_Type*self) ,
	void (*ptr_update_hook_func)(CheckDevice_Type*self) 
	):
	id(id),
    maxAllowTime(allow_time),
	priority(pri),
	is_offline_func(ptr_is_offline_func),
	state_changed_callback_func(ptr_state_changed_callback_func),
	update_hook_func(ptr_update_hook_func)
{}


/**
 * @brief 
 * 
 */
void app_sentry_CheckDevice_Init(void)
{
    CheckDeviceArry.checkDeviceNum=0;           //��ʼ���豸����
    for(int i=0;i<CheckDeviceID_EnumLength;i++)
    {
        CheckDeviceArry.deviceArry[i] = NULL;   //��ʼ���豸�б�
    }

    for(int i=0;i< CheckDeviceID_EnumLength+2 ; i++)    //��ʼ�������豸�б�Ϊȫ1
    {
        app_sentry_CheckDevice_OfflineList[i] = 1U;
    }
	
	if(											// ȷ�϶��д����ɹ�
		(QueueOfflineDevice = 
			xQueueCreate( CheckDeviceID_EnumLength,
						  sizeof(CheckDevice_Type*) )
		)
		==NULL
	)											
	{
        #ifdef APP_SENTRY_CHECK_DEVICE_DEBUG
        while(1);                               // ʧ����¶����
        #endif // APP_SENTRY_CHECK_DEVICE_DEBUG
    }									
	
    checkdevice_is_inited = 1;					// ����Ѿ���ʼ��
}






/**
 * @brief ��һ���豸�������� �����豸���������
 * 
 * @param     DeviceToAdd       ��Ҫ�����б����豸��ָ��
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd)
{
    if( ! checkdevice_is_inited )   //����Ƿ��ʼ��
        while(1){;}                    //��¶����

    for(int i=0;i<CheckDeviceArry.checkDeviceNum;i++)
    {
        if( DeviceToAdd->id == CheckDeviceArry.deviceArry[i]->id )  //�����غ�ID
        return HAL_ERROR;            								//�㱨���󣬲�����
    }

    CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum] = DeviceToAdd;
    return HAL_OK;
}





/**
 * @brief ͨ�����߼��������롣���������ӵ����Լ��ģ�ָ��̨����̵ģ�Taskʵ��֮�С���ע����Ҫ�ĺ궨�塣
 */
void app_sentry_CheckDevice_Handle(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //�����豸����
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry[tempDeviceId] );       //����豸���ߡ�����״̬д��
    }
}



/**
 * @brief       �Ӷ��л�ȡ�����豸
 * 
 * @param     device_id         �豸ID д������
 * @param     device_isoffline  �豸״̬д������
 * @return uint8_t      1Ϊ��ȡ�ɹ���0Ϊ���пջ��߻�ȡ����
 */
uint8_t app_sentry_CheckDevice_GetOfflineDeviceFromQueueTo(uint8_t* device_id, uint8_t* device_isoffline)
{
    CheckDevice_Type* device_ptr;
    if(xQueueReceive(QueueOfflineDevice,&device_ptr,0) == pdTRUE)
    {
        *device_id = device_ptr->id;
        *device_isoffline = app_sentry_CheckDevice_OfflineList[(uint8_t)device_ptr->id];

        device_ptr->is_change_reported = REPORT_NON;

		return 1U;
    }
	return 0U;
}

/**
 * @brief       ����CAN�յ��������豸��Ϣ
 * 
 * @param     ptrData ָ���յ����ݡ�����Э�飬[0]�ֽ�Ϊid��[1]�ֽ�Ϊ״̬
 */
void app_sentry_CheckDevice_CanRxCallback(uint8_t *ptrData)
{
    
    if(ptrData[0] < CheckDeviceID_EnumLength)   // ��������Χ
        app_sentry_CheckDevice_OfflineList[ ptrData[0] ] = (uint8_t)(ptrData[1]==1) ;   //д������״̬

}

