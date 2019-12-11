#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

uint8_t AmmoFeed::BlockedReaction(void)
{
	if(block->IsBlock)
	{
		if( !IsBlockInHandle )	//��ת��δ����
		{	//������ת�������
			softTargetAngle = RealAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//�趨��ת�Ƕ�
			Angle_Set(softTargetAngle);
			block->Clear_BlockFlag();	//�����ת��־
			IsBlockInHandle = 1;	//�������־
		}
		else //�ڴ����Զ�ת
		{
			IsBlockInHandle = 0;	//ǿ�ƻص�����
			block->Clear_BlockFlag();
		}
	}
	else
	{
		if(IsBlockInHandle)	//δ��ת���ڴ���
		{
			if( fabs(RealAngle - softTargetAngle) <=3 )	
			{
				IsBlockInHandle = 0;	//�ﵽ�ǶȺ󷵻�����
			}
			else
			{
				Angle_Set(softTargetAngle);	//��������
			}
		}
		else	//δ��ת��������ɣ�ֱ�ӷ���
		{	return 0; }
	}
	return 1;
}

void AmmoFeed::SpeedRun(void)
{
	if( BlockedReaction() )
	{ ;	}
	else
	{
		Speed_Set(rammingContiSpeed);
	}
	softTargetAngle = RealAngle;
	motor::CANSend();
}

void AmmoFeed::LocationRun(void)
{
	if( BlockedReaction() )
	{;}
	else
	{
		if( fabs(RealAngle - softTargetAngle ) <= 360.0f/feeder_division/3 && ( (HAL_GetTick() - rammerStepTime) > rammingDiscreDelay ) )
		{
			if(rammerStepLeft > 0)
			{
				softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//�趨��ת�Ƕȣ��������û��趨�ı��� ramDir ����
				rammerStepLeft-- ;
				rammerStepTime = HAL_GetTick();
			}
			if(rammerStepLeft == 0)
			{;}
//			if(rammerStepLeft < -1)	//����ģʽ
//			{
//				softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//�趨��ת�Ƕȣ��������û��趨�ı��� ramDir ����
//				rammerStepTime = HAL_GetTick();
//			}
		}
		else
		{;}
		Angle_Set(softTargetAngle);
	}
	motor::CANSend();
}

void AmmoFeed::Freefire(void)
{
	feed_mode = AMMOFEED_FREEFIRE;
	rammerStepLeft+=500;
	LocationRun();
}

void AmmoFeed::Once(void)
{
	feed_mode = AMMOFEED_ONCE;
	rammerStepLeft++;
	LocationRun();
}

void AmmoFeed::Burst(void)
{
	feed_mode = AMMOFEED_BURST;
	rammerStepLeft += BurstShootCnt;
	LocationRun();
}
void AmmoFeed::Stop(void)
{
	feed_mode = AMMOFEED_STOP;
	Speed_Set(0);
}
uint32_t t;
void AmmoFeed::FreeAndOnceManage(uint8_t trigger)
{
	static uint8_t act_flag=0;
	static uint32_t act_time_stamp;
	static uint8_t once_flag = 0;
	if( trigger )
	{
		if( act_flag == 0 ){
			act_flag= 1;	
			act_time_stamp = HAL_GetTick();
		}
	}
	else
	{	act_flag = 0;	}
	
	if(act_flag)
	{
		if( (t=HAL_GetTick()-act_time_stamp) > freefire_trig_time )	//����������ʱ
		{ Freefire(); }
		else
		{
			if(once_flag==0)	//�������
			{
				Once(); 
				once_flag =1;
			}
			LocationRun();
		}
	}
	else
	{
		Angle_Set(softTargetAngle);//���������softTargetAngle
		once_flag = 0;//��ձ�־
		rammerStepLeft = 0;//ONCE��ԭ�������Ӳ�������ղ�����ֹ�´δ���ֱ�Ӷ�
	}
	motor::CANSend();
}

/**
 * @brief  BURSTģʽ������
 * @param[in]  trigger �����־��1Ϊ����
 */

void AmmoFeed::BurstManage(uint8_t trigger)
{
	static uint8_t burst_flag =0;	//����BURST��־λ

	if( trigger )	//���������
	{
		if(burst_flag == 0)	//����־λδ��
		{	
			Burst();
			burst_flag = 1;	//����־������BURST
		}
		else
		{;}
		 LocationRun();		//Burst��LocationRunģʽ�²�������
	}
	else
	{
		burst_flag = 0;		//��ձ�־
		Angle_Set( softTargetAngle);	//���������softTargetAngle
		rammerStepLeft = 0;		//BURST��ԭ�������Ӳ�������ղ�����ֹ�´δ���ֱ�Ӷ�
	}
	motor::CANSend();
}









#undef SIGN

