/**
 * @file SentryCloudVision.hpp
  * @brief     ��̨�Ӿ�����ͨ��
  * @details  �ṩһ����̨������С�����������UARTͨѶ�����к������������ܵ�����Ϣ������-���棬��Ҫ���͵���Ϣ����������͡�
  * ����������app_vision��ע��app_vision������ζ�������ܹ�������������������ĺ����������㽫�����ܶ��**�ص�����**��д��
  * @author   ThunderDoge
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CLOUD_VISION_HPP_
#define __SENTRY_CLOUD_VISION_HPP_

#ifndef __PROJECT_SENTRY_CLOUD_     //���幤�̱�ʶ��__PROJECT_SENTRY_CLOUD_
#define __PROJECT_SENTRY_CLOUD_
#endif // __PROJECT_SENTRY_CLOUD_


///�������ļ�
#include "stm32f4xx.h"
#include "app_vision.hpp"
#include "SentryCloudCan.hpp"


/**
 * @brief �ڱ��Ӿ����ݻ���ṹ��
 * 
 */

extern sentry_vision_data VisionTx,VisionRx;    ///�����ýṹ��

extern void CloudVisonTxRoutine(void);  ///���߼��ص���������С��������һ��VisionTx��ȫ����Ϣ��
void CloudVisionSendFrame(uint8_t funcword,uint8_t* pData); ///����ʽUART���ͣ�������С��������һ������֡��

#endif // __SENTRY_CLOUD_VISION_HPP_












/**
 * @brief �Ӿ��������ݽ����ṹ��
 * @addtogroup Sentry_Vision
 */
// struct Sentry_vision_data
// {
//     uint8_t Frame_header = FRAME_HEADER_DATA;
//     uint8_t Frame_end = FRAME_END_DATA;
//     uint8_t Function_word; ///<����֡������
//     ///��������
//     float Vx;         ///<����X���ٶ�
//     float Vy;         ///<����Y���ٶ�
//     float Px;         ///<����X��·��
//     float Py;         ///<����Y��·��
//     float SpeedLimit; ///<��������
//     uint8_t pillar_flag;
//     uint8_t chassis_mode;
//     ///��̨����
//     float Yaw;          ///<Yaw��Ƕ�
// 	float YawSoft;
//     float Pitch;        ///<Pitch��Ƕ�
//     uint8_t Cloud_mode; ///<��̨ģʽ
//     uint8_t cloud_ctrl_mode;
//     ///�������
//     uint8_t Shoot_mode; ///<���ģʽ
//     float Shoot_speed;  ///<����ٶ�
//     uint8_t Shoot_freq; ///<���Ƶ��
//     ///���ݱ�־
//     uint32_t UpdateTime;
//     ///��־ϵͳʹ��
//     uint8_t Error_code = 0;          ///<�������
//     int16_t CAN1_motorlist = 0xffff; ///<CAN1����б�
//     int16_t CAN2_motorlist = 0xffff; ///<CAN2����б�
// };

///ROBOT_ERR �Ĵ������б�
