/**
  * @brief    �ڱ������߼�
  * @details  Encoding - GB2312
  * @author   
  * @date     
  * @version  
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#ifndef __SENTRY_CLOUD_LOGIC_HPP
#define __SENTRY_CLOUD_LOGIC_HPP

#include "SentryCloud.hpp"
#include "SentryCanCommu.hpp"
#include "bsp_vision.hpp"
#include "bsp_dbus.h"

enum GlobalModeName
{
    MODE_SAFE,
    MODE_MANUAL_SHOOTING_TEST,
    MODE_VIISON_SHOOTING_TEST,
    MODE_KEYBOARD_TEST,
    MODE_MANUAL_CHASSIS_MOVE,
    MODE_AUTONOMOUS,
};
enum CommandSourceName
{
	CMDSRC_DBUS,
	CMDSRC_CAN,
	CMDSRC_SELF,
};

extern GlobalModeName GlobalMode;
extern CommandSourceName CommandSource;


void ManualChassis();   //�ֶ�����
void ManualShoot();     //�ֶ��������
void VisionControl();   //�Ӿ�����
void AutoMove();        //ȫ�Զ��ƶ�
void GlobalSafe();      //��ȫģʽ

void ModeSelect();

void CloudCommuRoutine(void);


#endif // __SENTRY_CLOUD_LOGIC_HPP
