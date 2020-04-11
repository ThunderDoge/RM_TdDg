/**
 * @file task_SentryDownCloud.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Tasks of SentryCloud
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __TASK_SENTI_CLOUD_H_
#define __TASK_SENTI_CLOUD_H_

#ifndef __PROJECT_SENTRY_DOWN_CLOUD_     //���幤�̱�ʶ��__PROJECT_SENTRY_DOWN_CLOUD_
#define __PROJECT_SENTRY_DOWN_CLOUD_
#endif // __PROJECT_SENTRY_DOWN_CLOUD_

// �������ļ�

//ϵͳ����
#include "cmsis_os.h"

//Ӳ������
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "bsp_spi.h"
#include "app_imu.h"

//�������߼�
#include "Sentry.hpp"

void DownCloud_Init(void);  /// Ӳ����ʼ��
void TaskStarter(void); /// ȫ����������

#endif // __TASK_SENTI_CLOUD_H_
