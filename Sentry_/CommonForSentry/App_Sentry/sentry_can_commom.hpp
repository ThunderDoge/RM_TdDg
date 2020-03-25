/**
 * @file  sentry_can_commom.hpp
 * @brief    CANͨѶ������Ϣ
 * @details
 * �ڱ�CAN��صĺ���ȫ�������ڴ��ļ�֮�ڣ������޸�Э��ʱͳһ�޸ġ���̨�͵��̶������ڴ��ļ���
 * ���ǲ�������̨�͵��̶���Ҫ���ļ��еĵ�ȫ��������
 * ������̨�͵��̷ֱ���к궨�壺
 * __PROJECT_SENTRY_CLOUD_
 * __PROJECT_SENTRY_CHASSIS_
 * ���ļ�ʹ����������ֱ�������Ǹ�����Ҫ�ĺ�����
 * @author   ThunderDoge
 * @date     2020-2-20
 * @version  0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#ifndef __SENTRY_CAN_COMMOM_HPP_
#define __SENTRY_CAN_COMMOM_HPP_

//�������ļ�
#include "string.h"
#include "stm32f4xx_hal.h"
#include "bsp_can.hpp"         //��Ҫʹ��CAN
#include "sentry_config.hpp"   //�����˹��̱�ʶ�� __PROJECT_SENTRY_CLOUD_ ���� __PROJECT_SENTRY_CHASSIS_
#include "sentry_ctrl_def.hpp" //������MCUģʽö��ֵ

#ifdef __PROJECT_SENTRY_CLOUD_ //���ݹ��̱�ʶ�����ض�Ӧ�������ļ�
#include "SentryCloud.hpp"
#endif // __PROJECT_SENTRY_CLOUD_

#ifdef __PROJECT_SENTRY_CHASSIS_ //���ݹ��̱�ʶ�����ض�Ӧ   �������ļ�
#include "SentryChassis.hpp"
#endif //__PROJECT_SENTRY_CHASSIS_

//���붨��ĺ궨�塣���δ�������㶨��
#ifndef CAN_INTERBOARD
#define CAN_INTERBOARD hcan2
#endif // CAN_INTERBOARD

#define SENTRY_CAN_ID_CNT 20

/**
 * 2020-2-27 ThunderDoge�۲쵽�����������
 *
 *
 *
 * ��������ν���CAN��Ϣ��
 * ����CAN�Ľ��������Ѿ��������ã�ʹ�����������Լ���Ҫ�����ݵ�CANID
 * Ȼ��һ���ص���������Can�ص������б����ã�������CAN�Ѿ��յ���StdId������ָ��pData������˳�����һ���ڱ�
 * ͨ����CANID�б�[ SENTRY_CAN_ID_List
 * ]�������һ�����ϣ�������������ָ���б�[]�е������ͬλ�õ�ָ��(ָ����ö��[
 * __enum_sentry_can_id_RW_OrderWord ]��ʾ)�����浽����Ŀ���б�[]��ʾ�ĵ�ַ�С�
 * ����ÿ������Ҫͨ�Ź��ܡ�����Ҫ��
 * 1. ��� SENTRY_CAN_ID ��Ŀ
 * 2. ��� CANID�б�[] ��Ŀ
 * 3. ��� ͨѶָ���б���Ŀ
 * 4. ��� ��ȡ��ַ�б���Ŀ
 */
enum SENTRY_CAN_ID : uint32_t //���ͨѶID��
{
    //ͨ�ý��� 0x10X
    UP_CLOUD_STATES = 0X101U,                           ///< ����̨״̬��Ϣ
    DOWN_CLOUD_STATES = 0X102U,                         ///< ����̨״̬��Ϣ
    CHASSIS_STATES = 0X103U,                            ///< ����״̬��Ϣ
    CHASSIS_PILLAR = 0X104U,                            ///< ����ײ����Ϣ��֮���ĵ���
    OFFLINE_LIST =0X105U,                               ///< �����豸�б�
    SUPERIOR_SAFE = 0x106U,                             ///< ���̰�ȫ

    //����̨���� 0x11X
    SUPERIOR_UP_RELATIVE_CMD = 0X111U,                  ///< SUPERCMD���ϼ�ָ�����̨��Խǿ���
    SUPERIOR_UP_ABSOLUTE_CMD = 0X112U,                  ///< SUPERCMD����̨���ԽǶȿ���
    SUPERIOR_UP_SPEED_CMD = 0X113U,                     ///< ��δ���á�SUPERCMD����̨ת���ٶȿ���
    //����̨���� 0x12X
    SUPERIOR_DOWN_RELATIVE_CMD = 0X121U,                ///< SUPERCMD���ϼ�ָ�����̨��Խǿ���
    SUPERIOR_DOWN_ABSOLUTE_CMD = 0X122U,                ///< SUPERCMD����̨���ԽǶȿ���
    SUPERIOR_DOWN_SPEED_CMD = 0X123U,                   ///< ��δ���á�SUPERCMD����̨ת���ٶȿ���
    //���̽��� 0x13x
    SUPERIOR_CHASSIS_MOVE = 0X130U,                     ///< SUPERCMD�����ٶȿ���
    SUPERIOR_CHASSIS_SET_LOACTION = 0X131U,             ///< ����λ�ÿ���
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED = 0X132U, ///< ����λ�ÿ��Ƽ�����

};

// CanRx,CanTx��ʱ���������
#define __SENTRY_CAN_UPDATE_TIMESTAMP_COUNT 20 // CAN��������ж��ٸ�ʱ���

enum enumSentryCanUpdataTimestampPosition : int
{ // CAN�����ʱ������ö��
    tCanRecv = 0,
    tSuperiorControl,
    tSuperCon_UpCloud,
    tSuperCon_DownCloud,
    tSuperCon_Chassis,

    tUpCloud_Info,
    tUpCloud_Shoot_Info,
    tDownCloud_Info,
    tDownCloud_Shoot_Info,
    tChassis_Info,
    tChassis_Pil_Info,
};
struct CanCommuRecv_t /// CAN����ṹ�嶨��
{
    //ͨ����Ϣ
    uint32_t RecvId;
    uint32_t CanUpdateTime[__SENTRY_CAN_UPDATE_TIMESTAMP_COUNT];
    //�ϼ�ָ��
    // uint8_t SuperiorControlFlags;
    uint8_t SuperCon_CloudMode;
    uint8_t SuperCon_CloudFireOrder : 1;
    float SuperCon_PitchYaw[2];
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    //��̨��Ϣ
    float UpCloudPitchYaw[2];
    float DownCloudPitchYaw[2];
    uint8_t Shooting_Flag;
    //������Ϣ
    float Chassis_SpeedLocation[2];
    float Chassis_SpeedLimit;
    float Pillar_Dist;
    uint8_t Pillar_flag;
};
#undef __SENTRY_CAN_UPDATE_TIMESTAMP_COUNT //�ǵ�ȡ���ֲ���DEFINE

extern CanCommuRecv_t CanRx, CanTx; //����CAN����ṹ��

extern uint32_t
    SENTRY_CAN_ID_List[SENTRY_CAN_ID_CNT]; /// SENTRY_CAN_ID ��ƥ���б�

extern uint8_t SENTRY_CAN_ID_RwOrder_List[SENTRY_CAN_ID_CNT];

extern void *SENTRY_CAN_ID_RwPosition_List[SENTRY_CAN_ID_CNT][2];

extern uint8_t IS_SUPERIOR_VISION_CTRL, IS_SUPERIOR_MANUAL_CTRL;

// HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
//                                 uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
                                float argu1, float argu2);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id,
                                uint8_t *ptrData, size_t size=8);

/**
 * @addtogroup CAN_Interboard_Communication
 * @{
 */
/**
 * @addtogroup CAN_Interboard_COmmu_StatusBroudcast
 * @{
 */
void ChassisCanCommuRoutine(void);   ///���̶�ʱ���͵İ��CANͨ��
void UpCloudCanCommuRoutine(void);   ///��̨��ʱ���͵İ��CANͨ�� - ����̨
void DownCloudCanCommuRoutine(void); ///��̨��ʱ���͵İ��CANͨ�� - ����̨

void UP_CLOUD_STATES_CanRx(
    int StdId,
    uint8_t *
        ptrData);                                               ///�� CAN_INTERBOARD
                                                                ///��������̨����֡(UPCLOUD_STATES)���ݵ�CanRx����������ֲ�������ԡ�
void UP_CLOUD_STATES_CanTx();                                   ///�� CanTx ������ͨ�� CAN_INTERBOARD
                                                                ///��������̨����֡(UPCLOUD_STATES)
void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData); ///�������ơ�
void DOWN_CLOUD_STATES_CanTx();
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
void CHASSIS_STATES_CanTx();
/**
 * @}
 * CAN_Interboard_COmmu_StatusBroudcast
 */
/**
 * @addtogroup CAN_Interboard_Commu_SuperiorCommand
 * @{
 */
void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData);
void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
void UP_FEED_CanTx();
void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
void DOWN_FEED_CanTx();
void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_UP_RELATIVE_CMD_CanTx();
void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_UP_ABSOLUTE_CMD_CanTx();
void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_DOWN_RELATIVE_CMD_CanTx();
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx();
void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_CHASSIS_MOVE_CanTx();
void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId,
                                                     uint8_t *ptrData);
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx();
void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData);
void SUPERIOR_SAFE_CanTx();
/**
 * @}
 * CAN_Interboard_Commu_SuperiorCommand
 */
/**
 * @}
 * CAN_Interboard_Communication
 */

#endif // __SENTRY_CAN_COMMOM_HPP_
