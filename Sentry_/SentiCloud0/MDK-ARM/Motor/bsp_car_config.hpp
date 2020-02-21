/** 
* @file	   bsp_car_config.hpp
* @brief    �û��Զ������ͷ�ļ�
* @details  ��ͷ�ļ��µĲ�����Car_Driver����������йأ��û������������ã������ڴ����ÿ�ѡ��Ա�֤���ļ����Ķ� \n
*           ��û�г����µĲ�����ʱ�򣬲�Ҫ�ύ���ļ�����ר�ò����ύ��ͨ�ò�����
* @author   WMD
* @date     2019��1��17��19:58:05
* @version  0.1
* @par Copyright (c):  
*       WMD 
* @par ��־
*       V0.1	WMD������Ҫ����һ���ļ�����������ļ��ͱ������� CarDrv_config.hpp
*				V0.2	Evan-GH�޸�������ļ����ļ�������Ӧ����淶
*/  
#ifndef __BSP_CAR_CONFIG_HPP
#define __BSP_CAR_CONFIG_HPP

///ѡ��ʹ�õ�STM32��ͷ�ļ�
#include "stm32f4xx_hal.h"

///�Ƿ�ʹ���Զ����SYS������Ӱ�첿�ֱ�׼������printf��
//#define USE_OWM_SYS_DEF
//#include "usart.h"

#define LPF_NUM (0.3f)      //!<ǰ����ͨ�˲����Ĳ���
#define ChassisMax 15000    //!<������е��̵�����ٶ�

typedef enum{
    Stop=0x00U,              //!<���ֹͣ����
    Speed_Ctl=0x01U,         //!<���ʹ���ٶȻ�����
    Position_Ctl=0x02U,      //!<���ʹ��·�̻�����
    Gyro_Position_Ctl=0x03U, //!<����������ǻ����¹���(Cloud����ר��)
    Gyro_Speed_Ctl=0x04U,    //!<����������ǻ����¹���(Cloud����ר��)
    ///��������Ҫ��չ����ģʽʱ��Ҫ�������
}RunState_t;

//������̹�������
#define LIMIT_P (40)
#endif 
