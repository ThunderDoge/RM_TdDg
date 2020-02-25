/**
  * @file  sentry_can_commom.hpp
  * @brief    CANͨѶ������Ϣ
  * @details  �ڱ�CAN��صĺ���ȫ�������ڴ��ļ�֮�ڣ������޸�Э��ʱͳһ�޸ġ���̨�͵��̶������ڴ��ļ���
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
#include "stm32f4xx_hal.h"
#include "string.h"
#include "bsp_can.hpp"          //��Ҫʹ��CAN
#include "sentry_ctrl_def.hpp"  //������MCUģʽö��ֵ
#include "sentry_config.hpp"    //�����˹��̱�ʶ�� __PROJECT_SENTRY_CLOUD_ ���� __PROJECT_SENTRY_CHASSIS_


#ifdef __PROJECT_SENTRY_CLOUD_  //���ݹ��̱�ʶ�����ض�Ӧ�������ļ�
#include "SentryCloud.hpp"
#endif // __PROJECT_SENTRY_CLOUD_

#ifdef __PROJECT_SENTRY_CHASSIS_    //���ݹ��̱�ʶ�����ض�Ӧ�������ļ�
#include "SentryChassis.hpp"
#endif //__PROJECT_SENTRY_CHASSIS_

//���붨��ĺ궨�塣���δ�������㶨��
#ifndef CAN_INTERBOARD
#define CAN_INTERBOARD hcan2
#endif // CAN_INTERBOARD

enum SENTRY_CAN_ID : uint32_t //���ͨѶID��
{
    UP_CLOUD_STATES                             = 0X101U,   ///< ����̨״̬��Ϣ
    DOWN_CLOUD_STATES                           = 0X102U,   ///< ����̨״̬��Ϣ
    CHASSIS_STATES                              = 0X103U,   ///< ����״̬��Ϣ
    CHASSIS_PILLAR                              = 0X104U,   ///< ����ײ����Ϣ��֮���ĵ���
    SUPERIOR_UP_RELATIVE_CMD                    = 0X111U,   ///< SUPERCMD���ϼ�ָ�����̨��Խǿ���
    SUPERIOR_UP_ABSOLUTE_CMD                    = 0X112U,   ///< SUPERCMD����̨���ԽǶȿ���
	SUPERIOR_UP_SPEED_CMD                       = 0X113U,   ///< ��δ���á�SUPERCMD����̨ת���ٶȿ���
    SUPERIOR_DOWN_RELATIVE_CMD                  = 0X121U,   ///< SUPERCMD���ϼ�ָ�����̨��Խǿ���
    SUPERIOR_DOWN_ABSOLUTE_CMD                  = 0X122U,   ///< SUPERCMD����̨���ԽǶȿ���
	SUPERIOR_DOWN_SPEED_CMD                     = 0X123U,   ///< ��δ���á�SUPERCMD����̨ת���ٶȿ���
    SUPERIOR_CHASSIS_MOVE                       = 0X130U,   ///< SUPERCMD�����ٶȿ���
    SUPERIOR_CHASSIS_SET_LOACTION               = 0X131U,   ///< ����λ�ÿ���
    SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED   = 0X132U,   ///< ����λ�ÿ��Ƽ�����
    SUPERIOR_SAFE                               = 0x1A0U,   ///< ���̰�ȫ
};

struct CanCommuRecv_t
{
    uint32_t RecvId;
    uint8_t SuperiorControlFlags;
    float UpCloudPitchYaw[2];
    float DownCloudPitchYaw[2];
    float SuperCon_Relative_PitchYaw[2];
    float SuperCon_Absolute_PitchYaw[2];
    float Chassis_SpeedLocation[2];
    float Chassis_SpeedLimit;
    uint8_t feed_flag;
    uint32_t RecvUpdateTime;
    //������Ϣ
    uint8_t SuperCon_ChassisMode;
    float SuperCon_ChassisSpeedLocation[2];
    float Pillar_Dist;
    uint8_t Pillar_flag;
};

extern CanCommuRecv_t CanRx,CanTx;

extern uint8_t IS_SUPERIOR_VISION_CTRL,IS_SUPERIOR_MANUAL_CTRL;

HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2);

/**
 * @addtogroup CAN_Interboard_Communication
 * @{
 */
/**
 * @addtogroup CAN_Interboard_COmmu_StatusBroudcast
 * @{
 */
void ChassisCanCommuRoutine(void);  ///���̶�ʱ���͵İ��CANͨ��
void UpCloudCanCommuRoutine(void);  ///��̨��ʱ���͵İ��CANͨ�� - ����̨
void DownCloudCanCommuRoutine(void);    ///��̨��ʱ���͵İ��CANͨ�� - ����̨

void UP_CLOUD_STATES_CanRx(int StdId, uint8_t *ptrData);    ///�� CAN_INTERBOARD ��������̨����֡(UPCLOUD_STATES)���ݵ�CanRx����������ֲ�������ԡ�
void UP_CLOUD_STATES_CanTx();                               ///�� CanTx ������ͨ�� CAN_INTERBOARD ��������̨����֡(UPCLOUD_STATES)
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
void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData);
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
