/**
 * @file      app_sentry_check_device.cpp
 * @brief     离线检测功能，云台底盘公用文件。
 * @details   使用方法：
 * 1.如要添加新设备 在 CheckDeviceID_Enum 中加入设备ID枚举量
 * 2.定义设备对象 CheckDevice_Type
 * 3.初始化函数中调用 app_sentry_CheckDevice_Init()
 * 4.调用 app_sentry_CheckDevice_AddToArray() 将你定义的设备对象加到本机设备列表
 * 5.周期地运行 app_sentry_CheckDevice_Handle()
 * 6.在CAN回调函数中调用 app_sentry_CheckDevice_CanRxCallback() 以更新其他板上的设备的状态
 * 7.重要>>>>>>>>>>>>>>>>>>您需要自行定义 通过CAN发送本机离线设备的函数<<<<<<<<<<<<<<<<<<。 
 *   您可以通过 app_sentry_CheckDevice_GetOfflineDeviceFromQueueTo() 获取队列中待处理的离线设备
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.2
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "app_sentry_check_device.hpp"



//全局变量

static uint8_t checkdevice_is_inited=0;			//【已经初始化】标志变量

CheckDeviceArry_Type CheckDeviceArry;			//所有设备列表

QueueHandle_t QueueOfflineDevice;					/// 已离线设备列表 发送给通信函数处理

uint8_t app_sentry_CheckDevice_OfflineList[ CheckDeviceID_EnumLength+2 ] = {1};


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
        self->is_change_reported = REPORT_NEEDED;
    }
    app_sentry_CheckDevice_OfflineList[(uint8_t)self->id] =0;
}


/**
 * @brief   默认的离线回调函数。会把本设备 device 添加到队列 QueueOfflineDevice
 * 离线和上线都要
 * @param     device  指向自己的指针。离线回调函数将在初始化中 @see app_sentry_CheckDevice_Type_Init() 设为这个
 */
void Default_CheckDevice_OfflineCallbackFunc_AddToQueueToCommuTask(CheckDevice_Type* device)
{

        xQueueSendToBack( 	(QueueHandle_t) 	QueueOfflineDevice,
                        (CheckDevice_Type*)	device , 
                        (TickType_t)		100 / portTICK_PERIOD_MS 	);
        device->is_change_reported = REPORT_IN_QUEUE;

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
	{device->is_change_reported = REPORT_NEEDED;}
	
	if(device->is_change_reported == REPORT_NEEDED)
	{
		device->state_changed_callback_func(device);
		// device->is_change_reported = REPORT_IN_QUEUE;    //在 state_changed_callback_func 里面更改状态
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

    for(int i=0;i< CheckDeviceID_EnumLength+2 ; i++)    //初始化离线设备列表为全1
    {
        app_sentry_CheckDevice_OfflineList[i] = 1U;
    }
	
	if(											// 确认队列创建成功
		(QueueOfflineDevice = 
			xQueueCreate( CheckDeviceID_EnumLength,
						  sizeof(CheckDevice_Type*) )
		)
		==NULL
	)											
	{
        #ifdef APP_SENTRY_CHECK_DEVICE_DEBUG
        while(1);                               // 失败则暴露错误
        #endif // APP_SENTRY_CHECK_DEVICE_DEBUG
    }									
	
    checkdevice_is_inited = 1;					// 标记已经初始化
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
void app_sentry_CheckDevice_Handle(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //遍历设备数组
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry[tempDeviceId] );       //检查设备离线。离线状态写入
    }
}



/**
 * @brief       从队列获取离线设备
 * 
 * @param     device_id         设备ID 写到这里
 * @param     device_isoffline  设备状态写到这里
 * @return uint8_t      1为获取成功，0为队列空或者获取错误
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
 * @brief       处理CAN收到的离线设备消息
 * 
 * @param     ptrData 指向收到数据。根据协议，[0]字节为id，[1]字节为状态
 */
void app_sentry_CheckDevice_CanRxCallback(uint8_t *ptrData)
{
    
    if(ptrData[0] < CheckDeviceID_EnumLength)   // 检查参数范围
        app_sentry_CheckDevice_OfflineList[ ptrData[0] ] = (uint8_t)(ptrData[1]==1) ;   //写入离线状态

}


