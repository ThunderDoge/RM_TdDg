#ifndef	APP_AMMOFEED_H_
#define	APP_AMMOFEED_H_

#include "math.h"
#include "cmsis_os.h"
#include "Car_Driver.hpp"
#include "bsp_can.hpp"
//#include "universe.cpp"
#include "bsp_dbus.h"

typedef enum {
	AMMOFEED_FREEFIRE,
	AMMOFEED_BURST,
	AMMOFEED_ONCE,
	AMMOFEED_STOP,
}AMMOFEED_MODE;

class AmmoFeed : public softmotor
{
    public:
		AmmoFeed(uint8_t can_num,
                    uint16_t _can_id,
                    Motor_t *motor_type,
					uint8_t RamerDiv,
					int32_t rammingSpeed,
					int32_t rammingGap,
					int8_t	ramDir,
					float	rev_angle,
                    pid* PID_In,
                    pid* PID_Out=NULL,
                    uint16_t block_trigg_current = 4000,
                    uint16_t block_time = 200,
                    uint16_t block_release_angle = 2)
        :softmotor(can_num, _can_id, motor_type, PID_In, PID_Out) ,
		rammingContiSpeed(rammingSpeed) ,
		rammingDiscreDelay(rammingGap) ,
		feeder_division(RamerDiv) , 
		rev_angle_when_blocked(rev_angle) , 
		ramDir(ramDir)
		
        {
            Enable_Block(block_trigg_current , block_time , block_release_angle);
        }//!<���캯��
		AMMOFEED_MODE feed_mode;
		
		int32_t rammingContiSpeed = 0;		//�ٶȿ��Ƶ������ٶ�
		int32_t	rammingDiscreDelay = 0;		//·�̿��ƣ�ÿ������ͣ��ʱ��
		uint8_t feeder_division = 6;		//�����м�����
		uint32_t rammerStepTime = 0;		
		int32_t rammerStepLeft = 0;			//����ʣ�������·�̿��ƻ��������������
		float softTargetAngle = 0;			//��·�̽Ƕ��趨ֵ
		float rev_angle_when_blocked = 30;	//��תʱ��ת�ĽǶ�
		int8_t ramDir = 1;					//λ�ÿ��ƣ�ָʾת����������������������������
		uint16_t freefire_trig_time = 150;	//��ס�е������л���ʱ
		uint8_t BurstShootCnt = 3;			//N������N

		uint8_t BlockedReaction(void);	//��ת��Ӧ�������
		
		void SpeedRun(void);			//�ٶȿ���ģʽ
		void LocationRun(void);			//λ�ÿ���ģʽ
		
		void Freefire(void);			//����ģʽ
		void Burst(void);				//N����ģʽ��N=BurstShootCnt
		void Once(void);				//����ģʽ
		void Stop(void);				//ֹͣ
		
		void FreeAndOnceManage(uint8_t trigger);	//���µ�������ס����
		void BurstManage(uint8_t trigger);			//N��������
	private:
		uint8_t IsBlockInHandle=0;
};


#endif	//APP_RAMMER_H_
