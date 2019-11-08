/** 
* @brief    �����ļ�
* @details  �����������ͣ�������������
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/

#ifndef __MY_DEFINE_H_
#define __MY_DEFINE_H_

/*Cube���ɵ��ļ�*/
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "cmsis_os.h"

/*�弶֧�ְ�*/
#include "bsp_led.h"
#include "bsp_beep.h"
#include "bsp_can.h"
#include "bsp_dbus.h"
#include "bsp_spi.h"

/*Ӧ�ò��ļ�*/
#include "Application_pid.h"
#include "Application_oled.h"
#include "Application_math.h"
#include "Application_MPU9250.h"
#include "Application_DT.h"
#include "Application_filter.h"

/*�����ļ�*/
#include "Logtask.h"
#include "Controltask.h"
#include "Inittask.h"
#include "IMUtask.h"

/*ң�����ݽṹ��*/
typedef struct rc_rec
{
		int16_t CH_0;		/*ͨ��0*/
		int16_t CH_1;		/*ͨ��1*/
		int16_t CH_2;		/*ͨ��2*/
		int16_t CH_3;		/*ͨ��3*/
		uint8_t S1;			/*�󿪹�*/
		uint8_t S2;			/*�ҿ���*/
		int16_t Dial;		/*������*/
		struct 
		{
			int16_t X;				/*X��*/
			int16_t Y;				/*Y��*/
			int16_t Z;				/*Z��*/
			uint8_t Leftkey;	/*�Ҽ�*/
			uint8_t Rightkey;	/*�Ҽ�*/
		}Mouse;					/*�����Ϣ*/
		uint16_t Keys;	/*������Ϣ*/
}RC_Data;

/*����������ݽṹ��*/
typedef struct can_rec
{
		int16_t Loc;		/*���λ����Ϣ*/
		int16_t Rpm;		/*���ת����Ϣ*/
		int16_t Cur;		/*���������Ϣ*/
		int8_t Tem;			/*����¶���Ϣ*/
}Motor_Data;

/*���PID�����ṹ��*/
typedef struct motor_pid
{
		float Kp;							/*������*/
		float Ki;							/*������*/
		float Kd;							/*΢����*/
		int16_t I_Limit;			/*��������*/
		int16_t Output_Limit;	/*�������*/
}Motor_Pid;

/*�豸���߼��ṹ��*/
#define ONLINE 1
#define OFFLINE 0

typedef struct check_online
{
		int Dbus;							/*Dbus���߼��*/
		int CAN;							/*CAN1���߼��*/
		int MPU9250;					/*���������߼��*/
}Device_Check;

/*���ⲿ���õ����ݺͽṹ��*/
extern Motor_Data M3508_Data; /*3508������ݽ��սṹ��*/
extern RC_Data Dbus_Data;			/*Dbusң���ź����ݽ��սṹ��*/
extern Motor_Pid M3508_Pid;		/*M3508���PID�������ݽṹ��*/

extern uint8_t AK8963_ASA[3];

#define Init_Event	(0X01<<0) //��ʼ�������¼�

/*������*/
extern TaskHandle_t Log_Task_Handle;	/*ϵͳ����״̬������*/
extern TaskHandle_t Control_Task_Handle;	/*����������*/
extern TaskHandle_t Init_Task_Handle;	/*��ʼ��������*/
extern TaskHandle_t AppTaskCreat_Handle; /*���񴴽����*/
extern TaskHandle_t IMU_Task_Handle;	/*��̬����������*/

extern EventGroupHandle_t Inittask_Event_Handle; /*�ж�ϵͳ��ʼ���Ƿ���ɵ��¼�*/
extern SemaphoreHandle_t Dbus_Update_Handle; /*Dbus��ֵ�����±���*/
extern SemaphoreHandle_t CAN_Update_Handle; /*CAN��ֵ�����±���*/
extern SemaphoreHandle_t Dbus_Check_Handle; /*Dbus���߼���ֵ�ź���*/
extern SemaphoreHandle_t CAN_Check_Handle;	/*CAN���߼���ֵ�ź���*/

#endif
