#ifndef	APP_AMMOFEED_H_
#define	APP_AMMOFEED_H_

#include "math.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"

enum {
	AMMOFEED_FREEFIRE,
	AMMOFEED_BURST,
	AMMOFEED_FREEONCE
};


class AmmoFeed : public softmotor
{
    public:
			AmmoFeed(uint8_t can_num,
							 uint16_t _can_id,
							 Motor_t *motor_type,
							 uint8_t RamerDiv,
							 int8_t	ramDir,
							 pid* PID_In,
							 pid* PID_Out=NULL
							 )
					:softmotor(can_num, _can_id, motor_type, PID_In, PID_Out) ,
					 feeder_division(RamerDiv) , ramDir(ramDir){}//���캯��				
						 

						 
			int32_t rammingContiSpeed = 0;		//�ٶȿ��Ƶ������ٶ�
						 
			int32_t	rammingDiscreDelay = 10;		//·�̿��ƣ�ÿ������ͣ��ʱ�� ע��N����ģʽ�¼��ʱ����Ҫ�ϳ�
					// ͣ��ʱ��ֵ"2"���� 12-2-16��13-Eno
			uint8_t feeder_division = 7;		//�����м�����
					 
			int32_t rammerStepLeft = 0;			//����ʣ�ಽ����·�̿��ƻ��������������
										 
			float softTargetAngle = 0;			//��·�̽Ƕ��趨ֵ
					
						 
			float rev_angle_when_blocked = 30;	//��תʱ��ת�ĽǶ�,Ĭ��30��
			uint8_t IsBlockInHandle = 0;//���ڷ�ת��ת���
						 
			int8_t ramDir = 1;					//λ�ÿ��ƣ�ָʾת����������������������������
			
		  uint8_t BurstShootCnt = 3;			//N������N

			uint8_t BlockedReaction(void);	//��ת��Ӧ�������
			
			void PR_Handle(void); 			//����ִ�к���
			virtual void PositionRun(void);			//λ�ÿ���ģʽ
			
			void Freefire_Set(int32_t FreeSpeed);			//������תģʽ����
			void Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig);				//N����ģʽ����
			void FreeOnce_Set(int32_t	DiscreDelay,int16_t* trig);				//����ģʽ����
			int16_t* trigger;
			
		protected:		
			void Freefire(void);			//������תģʽ
			void Burst(void);				//N����ģʽ��N=BurstShootCnt
			void FreeOnce(void);		//��������
		private:
			uint8_t AmmoFeed_mode;		//����ģʽָʾ
		
			uint16_t freeonce_trig_time = 150;	//��ס�е����������л�����ʱʱ��
};


#endif	//APP_RAMMER_H_
