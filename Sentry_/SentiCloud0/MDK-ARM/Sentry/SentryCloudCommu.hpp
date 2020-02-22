/**
 * @file SentryCloudCommu.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __SENTRY_CLOUD_COMMU_HPP_
#define __SENTRY_CLOUD_COMMU_HPP_

#ifndef __PROJECT_SENTRY_CLOUD_     //���幤�̱�ʶ��__PROJECT_SENTRY_CLOUD_
#define __PROJECT_SENTRY_CLOUD_
#endif // __PROJECT_SENTRY_CLOUD_

///�������ļ�
#include "app_vision.hpp"
#include "SentryCloudVision.hpp"
#include "SentryCloudCan.hpp"
//CAN���ͨ�Ŷ��壬�����CAN�����޸Ĵ˴��궨��
#define CAN_INTERBOARD hcan2

//�˰���ʹ�����Ӿ�����
#define USE_VISION





/// �������������Է��ͺ���
extern void UpCloudCanCommuRoutine(void);
extern void CloudVisonTxRoutine(void);

extern sentry_vision_data VisionTx,VisionRx;    ///ͨ�����ݴ����

///�ص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);	//�ض����CAN�жϻص�����
#endif // __SENTRY_CLOUD_COMMU_HPP_




