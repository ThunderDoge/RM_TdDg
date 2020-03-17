/**
 * @file      app_sentry_check_device.
 * @brief     ���߼�⹦�ܣ���̨���̹����ļ���
 * @details   
 * @author   ThunderDoge
 * @date      2020-3-12
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
 */
#include "string.h"
#include "app_sentry_check_device.h"



//ȫ�ֱ���

static uint8_t checkdevice_is_inited=0;

CheckDeviceArry_Type CheckDeviceArry;

CheckParam_Type* LastOfflineDevice = NULL;      ///��һ�������豸
CheckParam_Type* CurrentOfflineDevice = NULL;   ///��ǰ�����豸



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
// void ObjDevice_PushStack(CheckDeviceID_Enum id,uint32_t* onLineTickPtr,uint16_t maxAllowTime,AlarmPriority_Enum priority,HAL_StatusTypeDef* status,FunctionalState cmd)
// {
// 	static uint8_t calledTimes = 0;	                               /* ������������ */
// 	if( onLineTickPtr == NULL )                                    // �������ָ��Ϊ�գ���¶����
// 	{
// 		while(1);
// 	}
	
// 	if( calledTimes == 0 )	                                       /* ��������ǵ�һ�α���������Ҫ��ʼ������ָ�� */
// 	{
// 		CheckDeviceArry.checkDeviceNum = -1;
// 	}
// 	calledTimes++;	
	
// 	CheckDeviceArry.checkDeviceNum++;                               //�����豸����
// 	if( CheckDeviceArry.checkDeviceNum < MaxCheckOfflineDevice )	 /* ����豸�����Ƿ����� */
// 	{
//         //�����豸����
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].id = id;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].lastTickPtr = onLineTickPtr;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].maxAllowTime = maxAllowTime;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].priority = priority;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].status = status;
// 		CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum].alarmCmd = cmd;
// 	}
// 	else    //�豸��������
// 	{
// 		CheckDeviceArry.checkDeviceNum = MaxCheckOfflineDevice - 1;  /* ά��ָ�����ݣ���ֹͣ���򣬽��б��� */
// 		while(1);
// 	}
// }	

/**
 * @brief 
 * 
 */
void CheckDevice_Init(void)
{
    CheckDeviceArry.checkDeviceNum=0;           //��ʼ���豸����
    for(int i=0;i<CheckDeviceID_EnumLength;i++)
    {
        CheckDeviceArry.deviceArry[i] = NULL;   //��ʼ��
    }
    checkdevice_is_inited = 1;
}


/**
 * @brief ��ʼ���豸�ṹ��
 * 
 * @param     device    �豸�ṹ���ָ��
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
 * @brief ��һ���豸�������� �����豸���������
 * 
 * @param     DeviceToAdd       ��Ҫ�����б���豸��ָ��
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef CheckDevice_AddToArray(CheckDevice_Type* DeviceToAdd)
{
    if( ! checkdevice_is_inited )   //����Ƿ��ʼ��
        while(1){;}                    //��¶����

    for(int i=0;i<CheckDeviceArry.checkDeviceNum;i++)
    {
        if( DeviceToAdd->id == CheckDeviceArry.deviceArry[i]->id )  //�����غ�ID
        return HAL_ERROR            //�㱨���󣬲����
    }

    CheckDeviceArry.deviceArry[CheckDeviceArry.checkDeviceNum] = DeviceToAdd;
    return HAL_OK;
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

    {
        if( device->is_offline_func != NULL )   //������������߼�⺯��
        {
            device->isOffline = device->is_offline_func;    //��ֱ��ִ�к���
        }
        else                                        //���û��
        {
            uint32_t dt = HAL_GetTick() - device->lastTick;     //��������ʱ��
            device->isOffline = (dt > device->maxAllowTime);    //д��״̬
        }
    }
}




/**
 * @brief ͨ�����߼��������롣��������ӵ����Լ��ģ�ָ��̨����̵ģ�Taskʵ��֮�С���ע����Ҫ�ĺ궨�塣
 */
void CheckDevice_TaskHandler(void)
{
    for( uint16_t tempDeviceId = 0;tempDeviceId < CheckDeviceArry.checkDeviceNum;tempDeviceId++ )  //�����豸����
    {
        DeviceOffline_Check( CheckDeviceArry.deviceArry + tempDeviceId );       //����豸���ߡ�����״̬д��
    }
}

