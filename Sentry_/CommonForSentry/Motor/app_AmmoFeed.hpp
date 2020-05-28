/**
* @file     app_AmmoFeed.hpp
* @brief    ͨ�ò��������ͷ�ļ�
* @details  
* @author    ThunderDoge, Asn
* @date      2020.5
* @version  1.0.7
* @copyright  RM2020��� 
* @par ��־
*		v1.0.0  2019/11/29  ʵ�ֻ�������\n
*		v1.0.1  2019/12/6   Asn�����˲��ּ򻯣�ʹ������ʹ��\n
*		v1.0.2  2019/12/13  �����˲��ֶ�����������Լ�һЩ�﷨���󡣲�����ת������Blocked_Reaction�Դ����������գ�����PID�������\n
*		v1.0.3  2019/12.27  ������ת�ж��߼�����ͳһ���ⲿcansend������Ȼ�����������ʹ����淶\n
*		v1.0.4  2019/12/31  ��ԭ����Pr_Handle������ֲ��Handle�У�ʹSet��ģʽ֮��ֻ��Ҫ����manager::CANSend()�ͺ���\n
*		v1.0.5  2020/1/21   ������ȫģʽ��bug��������ר��ֹͣģʽ��������trig�궨��\n
*		v1.0.6	2020/2/24	���ӣ���ÿ��trig������Դ���ֵ��0��Set_Step���������ⲿֱ�����ò���
*       v1.0.7  2020/5/22   ���� last_mode�����ʹ��staticʹ�ö��ammofeed��������ŵ����� 
*/ 
#ifndef	__APP_AMMOFEED_H
#define	__APP_AMMOFEED_H

#include "math.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"



//����ģʽö��
typedef enum {
	AMMOFEED_FREE_FIRE,//!<�������ģʽ��������������
	AMMOFEED_BURST,//!<N����ģʽ������������
	AMMOFEED_FREE_ONCE,//!<����������ģʽ
	AMMOFEED_STOP//!<ֹͣģʽ���ǰ�ȫģʽ
}FeedModeEnum;

///�����������
class AmmoFeed : public softmotor
{
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
					 feeder_division(RamerDiv) , rammer_direction(rammer_direction){}//���캯��				
						 
			void Set_Step(int32_t rammer_step)		//���ò������������ⲿֱ���趨�����ṩ�ӿ�
			{
				rammer_step_left = rammer_step;
			}							

			uint8_t Blocked_Reaction(void);			//��ת��Ӧ�������
			
			virtual void Step_Run(void);				//λ�ÿ���ģʽ
			virtual void Angle_Set(float Target_Angle);	//�Ƕ�����
			
			void Free_Fire_Set(int32_t FreeSpeed);								//������תģʽ����
			void Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig);	//N����ģʽ����
			void Free_Once_Set(int32_t	DiscreDelay,int16_t* trig);				//����ģʽ����
			void Stop_Set(void);												//ֹͣģʽ���ǰ�ȫ
			
			
			
		protected:	
			virtual void Handle(void);
	
			void Free_Fire(void);			//������תģʽ
			void Burst(void);				//N����ģʽ��N=burst_shoot_cnt
			void Free_Once(void);			//��������
		
		private:
			int32_t	ramming_discrete_delay = 10;	//·�̿��ƣ�ÿ������ͣ��ʱ�� ע��N����ģʽ�¼��ʱ����Ҫ�ϳ�
			// ͣ��ʱ��ֵ"2"���� 12-2-16��13-Eno
			uint8_t feeder_division = 7;			//�����м�����
					 
			int32_t rammer_step_left = 0;			//����ʣ�ಽ����·�̿��ƻ��������������
										 
			float soft_target_angle = 0;			//��·�̽Ƕ��趨ֵ
					
			float rev_angle_when_blocked = 30;		//��תʱ��ת�ĽǶ�,Ĭ��30��

			uint8_t is_block_in_handle = 0;			//���ڷ�ת��ת���
						 
			int8_t rammer_direction = 1;			//λ�ÿ��ƣ�ָʾת����������������������������
			
		    uint8_t burst_shoot_cnt = 3;			//N������N��Ĭ��������

			FeedModeEnum feed_mode;						//����ģʽָʾ
		
			FeedModeEnum last_mode;

			int16_t* trigger;						//Free_Once��Burst�Ĵ�������

			uint16_t free_once_trig_time = 150;		//��ס�е����������л�����ʱʱ��

            uint32_t block_cnt;
            uint32_t dual_block_cnt;
};


#endif
