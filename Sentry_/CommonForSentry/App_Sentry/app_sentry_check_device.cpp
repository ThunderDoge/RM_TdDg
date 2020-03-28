/**
 * @file      app_sentry_check_device.cpp
 * @brief     离线检测功能，云台底盘公用文件。
 * @details   
 * 1. 使用方法：
 *  【以下为编译时】
 *  1.1 添加你设备的ID在 @see CheckDeviceID_Enum 中
 *  1.2 定义你的 设备对象 @see CheckDevice_Type 使用其构造函数
 *      1.2.1 如果设备自带离线检测函数your_is_offline()，请将is_offline 函数指针给构造函数
 *      1.2.2 如果没有，请调用设备对象的 update_hook_func() 在设备数据更新时
 *      1.2.3 如果 设备自带离线检测函数 your_is_offline() 不符合设备对象中的函数指针的形式，你需要自己定义一个外套函数 套在 your_is_offline() 外面
 *  【以下为运行时，需要注意时序】
 *  1.3 运行时初始化函数中 初始化离线检测 使用 @see app_sentry_CheckDevice_Init()
 *  1.4 在1.3 执行完毕后，调用 app_sentry_CheckDevice_AddToArray() 将你的设备对象添加到 CheckDeviceArry
 *  1.5 在你的主要逻辑任务中（其他也可）调用 app_sentry_CheckDevice_TaskHandler 以运行功能
 *  1.6 在你的CAN通信任务中调用 app_sentry_CheckDevice_CommuTaskCallback
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "app_sentry_check_device.hpp"



//全局变量

static uint8_t checkdevice_is_inited=0;			//【已经初始化】标志变量

CheckDeviceArry_Type CheckDeviceArry;			// 本机包含的设备的列表

uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ] = {1U}; /// 哨兵所有设备的离线状态

QueueHandle_t QueueOfflineDeviceToCommuTask;					/// 已离线设备列表 发送给通信函数处理


/**
 * @brief       默认的更新钩子函数。更新钩子函数将在初始化中 @see app_sentry_CheckDevice_Type_Init() 设为这个
 * 
 * @param     self  来源
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
 * @brief   默认的离线回调函数。会把本设备 self 添加到队列 QueueOfflineDeviceToCommuTask
 * 离线和上线都要
 * @param     self  指向自己的指针。离线回调函数将在初始化中 @see app_sentry_CheckDevice_Type_Init() 设为这个
 */
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* self)
{
    xQueueSendToBack( 	(QueueHandle_t) 	QueueOfflineDeviceToCommuTask,
                    (CheckDevice_Type*)	self , 
                    (TickType_t)		100 / portTICK_PERIOD_MS 	);

}


/** 
  * @brief      器件离线判断函数
  * @param[in]  device 器件指针
  * @retval     None
  * @author     LittleDragon
  * @par         日志 
  *             2020-3-12   v0.1    ThunderDoge阅读了LittleDragon的RM2019英雄代码，觉得这一段很好，故采用
  */
static void DeviceOffline_Check(CheckDevice_Type* device)
{	
    if( device == NULL )    //无效的设备。很可能是越界
        while (1){;}        //暴露错误

	uint8_t last_offline = app_sentry_CheckDevice_OfflineList[(uint8_t)device->id];

	if( device->is_offline_func != NULL )   //如果有内置离线检测函数
	{
		app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] = device->is_offline_func();    //就直接执行函数
	}
	else                                        //如果没有
	{
		uint32_t dt = HAL_GetTick() - device->lastTick;     //计算离线时间
		app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] = (dt > device->maxAllowTime);    //写入状态
	}
	
	if(app_sentry_CheckDevice_OfflineList[(uint8_t)device->id] != last_offline)	// 登记：需要更新
	{device->is_change_reported = 0;}
	
	if(!device->is_change_reported)
	{
		device->state_changed_callback_func(device);
		device->is_change_reported = 1;
	}
    
}



/**
 * @brief     将一个设备参数对象 压入设备检测栈中
 * 
 * @param     id                器件编码(报警ID) @see CheckDeviceID_Enum
 * @param     onLineTickPtr     设备的在线时间变量指针。请用户自行更新。
 * @param     maxAllowTime      最大允许离线时长。超过此离线时长将被标记为离线
 * @param     priority          报警优先级
 * @param     status            状态指针，指向相关设备的状态标志位，通过指针进行修改. 可以为空.
 * @param     cmd               报警使能
 * @author     LittleDragon
 * @par         日志 
 *             2020-3-12   v0.1    ThunderDoge阅读了LittleDragon的RM2019英雄代码，觉得这一段很好，故采用
 */


/**
 * @brief Construct a new CheckDevice_Type::CheckDevice_Type object
 * 
 * @param     id    设备ID号  @see CheckDeviceID_Enum
 * @param     is_inter_brd              是否是板外设备
 * @param     pri                       优先级。不输入则默认
 * @param     ptr_is_offline_func       离线函数
 * @param     ptr_state_changed_callback_func   离线/上线回调函数
 * @param     ptr_update_hook_func              数据更新钩子函数
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
    CheckDeviceArry.checkDeviceNum=0;           //初始化设备个数
    for(int i=0;i<CheckDeviceID_EnumLength;i++)
    {
        CheckDeviceArry.deviceArry[i] = NULL;   //初始化设备列表
    }
	
	if(											// 确认队列创建成功
		(QueueOfflineDeviceToCommuTask = 
			xQueueCreate( CheckDeviceID_EnumLength,
						  sizeof(CheckDevice_Type*) )
		)
		==NULL
	)											
	{while(1);}									// 失败则暴露错误
	
    checkdevice_is_inited = 1;					// 标记已经初始化
}






/**
 * @brief 初始化设备结构体。全部设为默认值
 * 
 * @param     device    设备结构体的指针
 */
void app_sentry_CheckDevice_Type_Init(CheckDevice_Type* device)
{
	// 载入缺省参数
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
	// 初始化其结构体。
	app_sentry_CheckDevice_Type_Init(device);		
	
	//载入参数
	device->id = id;
	device->maxAllowTime = max_allow_time;
	device->priority = priority;
	device->alarm_enabled = enable_alarm;
	
	//函数指针 如果是NULL则仅使用缺省参数
	if(ptr_is_offline_func != NULL)
		device->is_offline_func = ptr_is_offline_func;
	if(ptr_update_hook_func != NULL)
		device->update_hook_func = ptr_update_hook_func;
	
	//载入
	if(app_sentry_CheckDevice_AddToArray(device) != HAL_OK)
	{
		
		#ifdef __APP_CHECK_DEVICE_DEBUG		// DEBUG用宏定义
		while(1){;}
		#endif	//__APP_CHECK_DEVICE_DEBUG
		
	}
	
}



/**
 * @brief 将一个设备参数对象 加入设备检测数组中
 * 
 * @param     DeviceToAdd       将要加入列表的设备的指针
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef app_sentry_CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd)
{
    if( ! checkdevice_is_inited )   //检查是否初始化
        while(1){;}                    //暴露错误

    for(int i=0;i<CheckDeviceArry.checkDeviceNum;i++)
    {
        if( DeviceToAdd->id == CheckDeviceArry.deviceArry[i]->id )  //发现重合ID
        return HAL_ERROR;            								//汇报错误，不添加
    }

    CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum] = DeviceToAdd;
    return HAL_OK;
}





/**
 * @brief 通用离线检测任务代码。可以在添加到你自己的（指云台或底盘的）Task实现之中。请注意需要的宏定义。
 */
void app_sentry_CheckDevice_TaskHandler(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //遍历设备数组
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry[tempDeviceId] );       //检查设备离线。离线状态写入
    }
}

/**
 * @brief 处理本机有 离线/上线设备时 该怎么办
 * 
 */
void app_sentry_CheckDevice_CommuTaskCallback(void)
{
    uint8_t data_to_send[2];
    CheckDevice_Type* device_to_send;

    if(xQueueReceive(QueueOfflineDeviceToCommuTask,&device_to_send,0U) == pdTRUE)    // 看看有东西没。延时为0 就是不用等的意思。不会阻塞。
    {
        data_to_send[0] = device_to_send->id;           //第一字节事ID
        data_to_send[1] = app_sentry_CheckDevice_OfflineList[ device_to_send->id ];   //第二字节事设备状态
        SentryCanSend(&CAN_INTERBOARD,(uint32_t)OFFLINE_LIST,(uint8_t)&data_to_send,(size_t)2U);   //发射
    }
}

void app_sentry_CheckDevice_CanRxCallBack(uint8_t* pdata)
{
    app_sentry_CheckDevice_OfflineList[ pdata[0] ] = pdata[1];  // 写入离线状态
}


