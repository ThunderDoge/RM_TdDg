/**
  * @file      app_sentry_diagnosis_commom.hpp
  * @brief     �ڱ������ϵͳ������豸�������������������ʾ��
  * @details   
  * @author   ThunderDoge
  * @date      2020-2-24
  * @version   0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */
#ifndef __APP_SENTRY_DIAGNOSIS_HPP_
#define __APP_SENTRY_DIAGNOSIS_HPP_
#include "stm32f4xx_hal.h"

#define APP_SENTRY_DIAG_FLAG_IN_BIT 1   //����ʶλ��bitʹ�á���ʶ�궨��,Ϊ1��ʾ��λʹ��,Ϊ0��ʾ���ֽ�ʹ��,�����ڶ�ȡ��д�뷽ʽ��ͬ.��Ӱ��ʵ��ʹ��.

#define APP_SENTRY_DIAG_GENARAL_DEVICE_OFFLINE_TIMEOUT_MS 1000; //һ���豸���ж�Ϊ���ߵ�ʱ�䣬��λ��ms
#define APP_SENTRY_DIAG_IS_OFFLINE_TIMEOUT(t0) ((HAL_GetTick()- (t0)) > APP_SENTRY_DIAG_GENARAL_DEVICE_OFFLINE_TIMEOUT_MS)

/**
 * @brief �ڱ��豸���룬�������ϵͳ��. DCODE_ stand for Devide_Code_ .
 * Sentry device code, used in diagnosis system.
 */
enum _sentry_device_code_enum:uint8_t { 
    DCODE_UPCLOUD_MCU = 0X00,
    DCODE_DOWNCLOUD_MCU,
    DCODE_CHASSIS_MCU,

    DCODE_UPCLOUD_CAMERA,
    DCODE_UPCLOUD_NUC,
    DCODE_UPCLOUD_FRIC_LEFT,
    DCODE_UPCLOUD_FRIC_RIGHT,
    DCODE_UPCLOUD_FEEDER,
    DCODE_UPCLOUD_PITCH,
    DCODE_UPCLOUD_YAW,
    DCODE_UPCLOUD_IMU,

    DCODE_DOWNCLOUD_CAMERA,
    DCODE_DOWNCLOUD_NUC,
    DCODE_DOWNCLOUD_FRIC_LEFT,
    DCODE_DOWNCLOUD_FRIC_RIGHT,
    DCODE_DOWNCLOUD_FEEDER,
    DCODE_DOWNCLOUD_PITCH,
    DCODE_DOWNCLOUD_YAW,
    DCODE_DOWNCLOUD_IMU,

    DCODE_CHASSIS_DRIVE,
    DCODE_CHASSIS_JUDGE,
    DCODE_CHASSIS_PILLARWATCH_LEFT,
    DCODE_CHASSIS_PILLARWATCH_RIGHT,
};

#if APP_SENTRY_DIAG_FLAG_IN_BIT
    #define APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE 5
#elif
    #define APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE 40
#endif

extern uint8_t OfflineListBytes[APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE];

#define _APP_SENTRY_DIAG_SET_DEVICE_AS(device_code,online_states) 

void app_sentry_diag_SetDevice(uint8_t device_code, int online_states);

#endif // __APP_SENTRY_DIAGNOSIS_HPP_


