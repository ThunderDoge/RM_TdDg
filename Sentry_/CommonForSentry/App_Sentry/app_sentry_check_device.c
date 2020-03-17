/**
 * @file      app_sentry_check_device.
 * @brief     离线检测功能，云台底盘公用文件。
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "string.h"
#include "app_sentry_check_device.h"



//全局变量

static uint8_t checkdevice_is_inited=0;

CheckDeviceArry_Type CheckDeviceArry;

CheckParam_Type* LastOfflineDevice = NULL;      ///上一个离线设备
CheckParam_Type* CurrentOfflineDevice = NULL;   ///当前离线设备



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
// void ObjDevice_PushStack(CheckDeviceID_Enum id,uint32_t* onLineTickPtr,uint16_t maxAllowTime,AlarmPriority_Enum priority,HAL_StatusTypeDef* status,FunctionalState cmd)
// {
// 	static uint8_t calledTimes = 0;	                               /* 函数被调次数 */
// 	if( onLineTickPtr == NULL )                                    // 如果在线指针为空，暴露错误
// 	{
// 		while(1);
// 	}
	
// 	if( calledTimes == 0 )	                                       /* 如果函数是第一次被调，则需要初始化队列指针 */
// 	{
// 		CheckDeviceArry.checkDeviceNum = -1;
// 	}
// 	calledTimes++;	
	
// 	CheckDeviceArry.checkDeviceNum++;                               //增加设备数量
// 	if( CheckDeviceArry.checkDeviceNum < MaxCheckOfflineDevice )	 /* 检测设备数组是否已满 */
// 	{
//         //载入设备参数
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].id = id;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].lastTickPtr = onLineTickPtr;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].maxAllowTime = maxAllowTime;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].priority = priority;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].status = status;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].alarmCmd = cmd;
// 	}
// 	else    //设备数组已满
// 	{
// 		CheckDeviceArry.checkDeviceNum = MaxCheckOfflineDevice - 1;  /* 维护指向数据，并停止程序，进行报错 */
// 		while(1);
// 	}
// }	

/**
 * @brief 
 * 
 */
void CheckDevice_Init(void)
{
    CheckDeviceArry.checkDeviceNum=0;           //初始化设备个数
    for(int i=0;i<CheckDeviceID_EnumLength;i++)
    {
        CheckDeviceArry.deviceArry[i] = NULL;   //初始化
    }
    checkdevice_is_inited = 1;
}


/**
 * @brief 初始化设备结构体
 * 
 * @param     device    设备结构体的指针
 */
void CheckDevice_Type_Init(CheckDevice_Type* device)
{
    device->lastTick = 0;
    device->maxAllowTime = 0;
    device->isOffline = 0;
    device->priority = PriorityNormal;
    device->is_offline_func = NULL;
}



/**
 * @brief 将一个设备参数对象 加入设备检测数组中
 * 
 * @param     DeviceToAdd       将要加入列表的设备的指针
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd)
{
    if( ! checkdevice_is_inited )   //检查是否初始化
        while(1){;}                    //暴露错误

    for(int i=0;i<CheckDeviceArry.checkDeviceNum;i++)
    {
        if( DeviceToAdd->id == CheckDeviceArry.deviceArry[i]->id )  //发现重合ID
        return HAL_ERROR            //汇报错误，不添加
    }

    CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum] = DeviceToAdd;
    return HAL_OK;
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

    {
        if( device->is_offline_func != NULL )   //如果有内置离线检测函数
        {
            device->isOffline = device->is_offline_func;    //就直接执行函数
        }
        else                                        //如果没有
        {
            uint32_t dt = HAL_GetTick() - device->lastTick;     //计算离线时间
            device->isOffline = (dt > device->maxAllowTime);    //写入状态
        }
    }
}




/**
 * @brief 通用离线检测任务代码。可以在添加到你自己的（指云台或底盘的）Task实现之中。请注意需要的宏定义。
 */
void CheckDevice_TaskHandler(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //遍历设备数组
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry + tempDeviceId );       //检查设备离线。离线状态写入
    }
}

