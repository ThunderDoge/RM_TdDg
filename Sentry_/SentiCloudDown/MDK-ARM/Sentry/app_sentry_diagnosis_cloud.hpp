/**
 * @file      app_sentry_diagnosis_cloud.hpp
 * @brief     �ڱ������ϵͳ_��̨�ֲ�
 * @details   
 * @author   ThunderDoge
 * @date      
 * @version   
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                         Using encoding: gb2312
*/
#ifndef __APP_SENTRY_DIAGNOSIS_CLOUD_HPP_
#define __APP_SENTRY_DIAGNOSIS_CLOUD_HPP_

//���̱�־�궨��
 #include "sentry_config.hpp"
 //�������ļ�
#include "bsp_motor.hpp"
#include "app_sentry_diagnosis_commom.hpp"
#include "SentryCloud.hpp"
#include "app_vision.hpp"

extern uint8_t OfflineListBytes[APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE];

void app_sentry_diag_cloud_Init();
void app_sentry_diag_cloud_Handle();

#endif // __APP_SENTRY_DIAGNOSIS_CLOUD_HPP_




 