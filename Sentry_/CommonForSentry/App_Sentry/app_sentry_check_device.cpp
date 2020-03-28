/**
 * @file      app_sentry_check_device.cpp
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   
 * 1. ʹ�÷�����
 *  ������Ϊ����ʱ��
 *  1.1 ������豸��ID�� @see CheckDeviceID_Enum ��
 *  1.2 ������� �豸���� @see CheckDevice_Type ʹ���乹�캯��
 *      1.2.1 ����豸�Դ����߼�⺯��your_is_offline()���뽫is_offline ����ָ������캯��
 *      1.2.2 ���û�У�������豸����� update_hook_func() ���豸���ݸ���ʱ
 *      1.2.3 ��� �豸�Դ����߼�⺯�� your_is_offline() �������豸�����еĺ���ָ�����ʽ������Ҫ�Լ�����һ�����׺��� ���� your_is_offline() ����
 *  ������Ϊ����ʱ����Ҫע��ʱ��
 *  1.3 ����ʱ��ʼ�������� ��ʼ�����߼�� ʹ�� @see app_sentry_CheckDevice_Init()
 *  1.4 ��1.3 ִ����Ϻ󣬵��� app_sentry_CheckDevice_AddToArray() ������豸������ӵ� CheckDeviceArry
 *  1.5 �������Ҫ�߼������У�����Ҳ�ɣ����� app_sentry_CheckDevice_TaskHandler �����й���
 *  1.6 �����CANͨ�������е��� app_sentry_CheckDevice_CommuTaskCallback
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "app_sentry_check_device.hpp"



//ȫ�ֱ���

static uint8_t checkdevice_is_inited=0;			//���Ѿ���ʼ������־����

CheckDeviceArry_Type CheckDeviceArry;			// �����������豸���б�

uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ] = {1U}; /// �ڱ������豸������״̬

QueueHandle_t QueueOfflineDeviceToCommuTask;					/// �������豸�б� ���͸�ͨ�ź�������


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
        self->is_change_reported = 0;
    }
    app_sentry_CheckDevice_OfflineList[(uint8_t)self->id] =0;
}


/**
 * @brief   Ĭ�ϵ����߻ص���������ѱ��豸 self ��ӵ����� QueueOfflineDeviceToCommuTask
 * ���ߺ����߶�Ҫ
 * @param     self  ָ���Լ���ָ�롣���߻ص��������ڳ�ʼ���� @see app_sentry_CheckDevice_Type_Init() ��Ϊ���
 */
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* self)
{
    xQueueSendToBack( 	(QueueHandle_t) 	QueueOfflineDeviceToCommuTask,
                    (CheckDevice_Type*)	self , 
                    (TickType_t)		100 / portTICK_PERIOD_MS 	);

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
	{device->is_change_reported = 0;}
	
	if(!device->is_change_reported)
	{
		device->state_changed_callback_func(device);
		device->is_change_reported = 1;
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
	
	if(											// ȷ�϶��д����ɹ�
		(QueueOfflineDeviceToCommuTask = 
			xQueueCreate( CheckDeviceID_EnumLength,
						  sizeof(CheckDevice_Type*) )
		)
		==NULL
	)											
	{while(1);}									// ʧ����¶����
	
    checkdevice_is_inited = 1;					// ����Ѿ���ʼ��
}






/**
 * @brief ��ʼ���豸�ṹ�塣ȫ����ΪĬ��ֵ
 * 
 * @param     device    �豸�ṹ���ָ��
 */
void app_sentry_CheckDevice_Type_Init(CheckDevice_Type* device)
{
	// ����ȱʡ����
	device->id = CheckDeviceID_EnumLength;
    device->lastTick = 0;
    device->maxAllowTime = 0;
    app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] = 0;
    device->priority = PriorityNormal;
    device->is_offline_func = NULL;
    device->update_hook_func = Default_CheckDevice_UpdateHookFunc;
}

void app_sentry_CheckDevice_Type_Init_AddToArray(	CheckDevice_Type* 	device,
										CheckDeviceID_Enum 	id,
										uint16_t 			max_allow_time,
										AlarmPriority_Enum 	priority,
										FunctionalState		enable_alarm,
										uint8_t (*ptr_is_offline_func)(void),
										void (*ptr_update_hook_func)(CheckDevice_Type*self) )
{
	// ��ʼ����ṹ�塣
	app_sentry_CheckDevice_Type_Init(device);		
	
	//�������
	device->id = id;
	device->maxAllowTime = max_allow_time;
	device->priority = priority;
	device->alarm_enabled = enable_alarm;
	
	//����ָ�� �����NULL���ʹ��ȱʡ����
	if(ptr_is_offline_func != NULL)
		device->is_offline_func = ptr_is_offline_func;
	if(ptr_update_hook_func != NULL)
		device->update_hook_func = ptr_update_hook_func;
	
	//����
	if(app_sentry_CheckDevice_AddToArray(device) != HAL_OK)
	{
		
		#ifdef __APP_CHECK_DEVICE_DEBUG		// DEBUG�ú궨��
		while(1){;}
		#endif	//__APP_CHECK_DEVICE_DEBUG
		
	}
	
}



/**
 * @brief ��һ���豸�������� �����豸���������
 * 
 * @param     DeviceToAdd       ��Ҫ�����б���豸��ָ��
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd)
{
    if( ! checkdevice_is_inited )   //����Ƿ��ʼ��
        while(1){;}                    //��¶����

    for(int i=0;i<CheckDeviceArry.checkDeviceNum;i++)
    {
        if( DeviceToAdd->id == CheckDeviceArry.deviceArry[i]->id )  //�����غ�ID
        return HAL_ERROR;            								//�㱨���󣬲����
    }

    CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum] = DeviceToAdd;
    return HAL_OK;
}





/**
 * @brief ͨ�����߼��������롣��������ӵ����Լ��ģ�ָ��̨����̵ģ�Taskʵ��֮�С���ע����Ҫ�ĺ궨�塣
 */
void app_sentry_CheckDevice_TaskHandler(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //�����豸����
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry[tempDeviceId] );       //����豸���ߡ�����״̬д��
    }
}

/**
 * @brief �������� ����/�����豸ʱ ����ô��
 * 
 */
void app_sentry_CheckDevice_CommuTaskCallback(void)
{
    uint8_t data_to_send[2];
    CheckDevice_Type* device_to_send;

    if(xQueueReceive(QueueOfflineDeviceToCommuTask,&device_to_send,0U) == pdTRUE)    // �����ж���û����ʱΪ0 ���ǲ��õȵ���˼������������
    {
        data_to_send[0] = device_to_send->id;           //��һ�ֽ���ID
        data_to_send[1] = app_sentry_CheckDevice_OfflineList[ device_to_send->id ];   //�ڶ��ֽ����豸״̬
        SentryCanSend(&CAN_INTERBOARD,(uint32_t)OFFLINE_LIST,(uint8_t)&data_to_send,(size_t)2U);   //����
    }
}

void app_sentry_CheckDevice_CanRxCallBack(uint8_t* pdata)
{
    app_sentry_CheckDevice_OfflineList[ pdata[0] ] = pdata[1];  // д������״̬
}


