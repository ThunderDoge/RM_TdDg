/**
  * @file       app_AmmoFeed.hpp
  * @brief    ͨ�ò��������
  * @details  
  * @author   ThunderDoge & Asn
  * @date     2019/12/13
  * @version  v1.0.3 
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  * v1.0.0  2019/11/29  ʵ�ֻ�������
  * v1.0.1  2019/12/6   Asn�����˲��ּ򻯣�ʹ������ʹ�á�
  * v1.0.2  2019/12/13  �����˲��ֶ�����������Լ�һЩ�﷨���󡣲�����ת������Blocked_Reaction�Դ����������գ�����PID���������
	*	v1.0.3  2019/12.27  ������ת�ж��߼�����ͳһ���ⲿcansend������Ȼ�����������ʹ����淶
  */

#ifndef	__APP_AMMOFEED_H
#define	__APP_AMMOFEED_H

#include "math.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"
#include "app_math.h"

/**
 * @brief AmmoFeed������ģʽ ö����
 * 
 */
enum ammofeed_status_enum {
	AMMOFEED_FREE_FIRE,
	AMMOFEED_BURST,
	AMMOFEED_FREE_ONCE,
};


class AmmoFeed : public softmotor
{
    friend class SentryCloud;   ///ʹ��SentryCloud���Ե��ù������ƺ���
    friend class SentryChassis;     
    public:
			AmmoFeed(uint8_t can_num,
							 uint16_t _can_id,
							 Motor_t *motor_type,
							 uint8_t RamerDiv,
							 int8_t	rammer_direction,
							 pid* PID_In,
							 pid* PID_Out=NULL
							 )
					:softmotor(can_num, _can_id, motor_type, PID_In, PID_Out) ,
					 feeder_division(RamerDiv) , rammer_direction(rammer_direction){}///<���캯��				
						 

						 
			int32_t ramming_speed = 0;		///<PID�ٶȻ���Ŀ���ٶȡ�
						 
			int32_t	ramming_discrete_delay = 10;		//·�̿���ʱ��ÿ������ͣ��ʱ�� ע��N����ģʽ�¼��ʱ����Ҫ�ϳ�
					// ͣ��ʱ��ֵ"2"���� 12-2-16��13-Eno
			uint8_t feeder_division = 7;		//�������м�����
					 
			int32_t rammer_step_left = 0;			//����ʣ�ಽ����·�̿��ƻ��������������
										 
			float soft_target_angle = 0;			//��·�̽Ƕ��趨ֵ
					
						 
			float rev_angle_when_blocked = 20;	//��תʱ��ת�ĽǶ�,Ĭ��20��
						 
			int8_t rammer_direction = 1;					//λ�ÿ��ƣ�ָʾת����������������������������
			
		    uint8_t burst_shoot_cnt = 3;			//N������N

			uint8_t Blocked_Reaction(void);	//��ת��Ӧ�������
			
			virtual void Step_Run(void);			//λ�ÿ���ģʽ
		
			virtual void  Safe_Set(void) override;
			
			int16_t trigger;//Free_Once��Burst�Ĵ�������
			uint8_t feed_mode;		//����ģʽָʾ
			uint16_t free_once_trig_time = 150;	//��ס�е����������л�����ʱʱ��
			
		protected:	
			void Free_Fire_Set(int32_t FreeSpeed);			//������תģʽ����
			void Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig);				//N����ģʽ����
			void Free_Once_Set(int32_t	DiscreDelay,int16_t trig);				//����ģʽ����
			virtual void Handle(void);
	
			void Free_Fire(void);			//������תģʽ
			void Burst(void);				//N����ģʽ��N=burst_shoot_cnt
			void Free_Once(void);		    //��������
            //�ڲ�״̬����
			uint8_t is_block_in_handle = 0;     //���ڷ�ת��ת���
		
};


#endif
