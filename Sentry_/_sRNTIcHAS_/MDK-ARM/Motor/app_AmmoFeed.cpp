#include "app_AmmoFeed.hpp"
#ifndef SIGN
#define SIGN(x) ((x)>0?1:((x)<0?-1:1))
#endif

void AmmoFeed::PR_Handle(void)
{
	static uint8_t last_mode;
if(!
		BlockedReaction())//��תʱ���ж�ת����
	{
	if(AmmoFeed_mode != last_mode)
	{
		softTargetAngle = SoftAngle;
	}
	switch(AmmoFeed_mode)
	{
		case AMMOFEED_FREEFIRE:
			Freefire();
			break;
		case AMMOFEED_BURST:
			Burst();
			break;
		case AMMOFEED_FREEONCE:
			FreeOnce();
			break;
		default:
			Safe_Set();
			break;
	}
	}
	last_mode = AmmoFeed_mode;
}


uint8_t AmmoFeed::BlockedReaction(void)
{
	if(block == NULL)
		return 0;
	if(block->IsBlock)
	{
		if( !IsBlockInHandle )	//��ת��δ����
		{	//������ת�������
			softTargetAngle = SoftAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//�趨��ת�Ƕ�
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
			if( fabs(SoftAngle - softTargetAngle) <=3 )	
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

/** 
    * @brief ���������еĺ���
*/
void AmmoFeed::PositionRun(void)
{
	static int16_t Step_Overflow;
	static uint32_t rammerStepTime = 0;
	if(( (HAL_GetTick() - rammerStepTime) > rammingDiscreDelay ) ) 
	{
		if(rammerStepLeft > 0)//������ڲ���
		{
			softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//�趨��ת�Ƕȣ��������û��趨�ı��� ramDir ����
			rammerStepLeft-- ;
			rammerStepTime = HAL_GetTick();
		}
		else if(rammerStepLeft == 0)//�������Ϊ0
		{
			Step_Overflow = (int16_t)floorf((softTargetAngle - SoftAngle)/(360.0f/feeder_division)/SIGN(ramDir));
			if(Step_Overflow<=0)Step_Overflow = 0;
			softTargetAngle -= ( SIGN(ramDir) * 360.0f/feeder_division)*Step_Overflow;
		}
	}
	Angle_Set(softTargetAngle);
	motor::CANSend();
}
/** 
* @brief  ���ɿ���ģʽ���ú���
* @param[in]   �����ٶ�
* @retval  
* @par ��־ 
*
*/
void AmmoFeed::Freefire_Set(int32_t FreeSpeed)
{
	AmmoFeed_mode = AMMOFEED_FREEFIRE;
	rammingContiSpeed = FreeSpeed;
}
/** 
* @brief  N����ģʽ���ú���
* @param[in]   DiscreDelay ÿ��һ�����ʱ��   trig N�����Ĵ������� 
* @retval   
* @par ��־ 
*
*/
void AmmoFeed::Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig)
{
	AmmoFeed_mode = AMMOFEED_BURST;
	BurstShootCnt = ShootCnt;
	rammingDiscreDelay = DiscreDelay;
	trigger = trig;
}
/** 
* @brief  ��������ģʽ���ú���
* @param[in]   DiscreDelay ÿ��һ�����ʱ��   trig �����Ĵ������� 
* @retval   
* @par ��־ 
*
*/
void AmmoFeed::FreeOnce_Set(int32_t	DiscreDelay,int16_t* trig)
{
	AmmoFeed_mode = AMMOFEED_FREEONCE;
	rammingDiscreDelay = DiscreDelay;
	trigger = trig;
}

/** 
    * @brief ���ɿ���ģʽ
*/
void AmmoFeed::Freefire(void)
{
	Speed_Set(rammingContiSpeed);
	softTargetAngle = RealAngle;
	motor::CANSend();
}


/** 
    * @brief ��������ģʽ
*/
void AmmoFeed::FreeOnce(void)
{
	static uint8_t act_flag=0;
	static uint32_t act_time_stamp;
	static uint8_t once_flag = 0;

	if( *trigger>200 )
	{
		if( act_flag == 0 )
		{
			act_flag= 1;
			act_time_stamp = HAL_GetTick();
		}
	}
	else
	{	
		act_flag = 0;	
	}
	
	if(act_flag)
	{
		if((HAL_GetTick()-act_time_stamp)>freeonce_trig_time)
		{
			rammerStepLeft++;			
		}
		else
		{
			if(once_flag == 0)
			{
				once_flag = 1;
			}
		}
	}
	else
	{
		if(once_flag == 1)
		{
			rammerStepLeft = 1;//���η��䣬�͸���һ������
			once_flag = 0;
		}
		else
		{
			rammerStepLeft = 0;//������ʱ�Ѳ�������
		}
	}
	PositionRun();//���������в���
}


/** 
    * @brief N����ģʽ
*/
void AmmoFeed::Burst(void)
{
	static uint8_t burst_flag =0;	//����BURST��־λ
	
	if( *trigger>200 )	//���������
	{
		if(burst_flag == 0)	//����־λδ��
		{	
			rammerStepLeft = BurstShootCnt;
			burst_flag = 1;	//����־������BURST
		}
	}
	else
	{
		burst_flag = 0;
	}
	PositionRun();
}









#undef SIGN

