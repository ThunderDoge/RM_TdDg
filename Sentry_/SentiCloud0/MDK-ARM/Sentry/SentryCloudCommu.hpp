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

///�������ļ�
#include "app_vision.hpp"
#include "SentryCloudVision.hpp"
#include "SentryCloudCan.hpp"
//CAN���ͨ�Ŷ��壬�����CAN�����޸Ĵ˴��궨��
#define CAN_INTERBOARD hcan2

//�˰���ʹ�����Ӿ�����
#define USE_VISION


extern Sentry_vision_data VisionTx,VisionRx;    ///ͨ�����ݴ����



/// �������������Է��ͺ���
extern void CloudCanCommuRoutine(void);
extern void CloudVisionRoutine(void);
///�����ýṹ��



#endif // __SENTRY_CLOUD_COMMU_HPP_




