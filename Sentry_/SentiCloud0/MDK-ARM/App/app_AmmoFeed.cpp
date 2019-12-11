#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

uint8_t AmmoFeed::BlockedReaction(void)
{
	if(block->IsBlock)
	{
		if( !IsBlockInHandle )	//堵转，未处理
		{	//正常堵转处理程序
			softTargetAngle = RealAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//设定反转角度
			Angle_Set(softTargetAngle);
			block->Clear_BlockFlag();	//清楚堵转标志
			IsBlockInHandle = 1;	//立起处理标志
		}
		else //在处理，仍堵转
		{
			IsBlockInHandle = 0;	//强制回到正常
			block->Clear_BlockFlag();
		}
	}
	else
	{
		if(IsBlockInHandle)	//未堵转，在处理。
		{
			if( fabs(RealAngle - softTargetAngle) <=3 )	
			{
				IsBlockInHandle = 0;	//达到角度后返回正常
			}
			else
			{
				Angle_Set(softTargetAngle);	//继续处理
			}
		}
		else	//未堵转，处理完成，直接返回
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
				softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//设定反转角度，方向由用户设定的变量 ramDir 决定
				rammerStepLeft-- ;
				rammerStepTime = HAL_GetTick();
			}
			if(rammerStepLeft == 0)
			{;}
//			if(rammerStepLeft < -1)	//无穷模式
//			{
//				softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//设定反转角度，方向由用户设定的变量 ramDir 决定
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
		if( (t=HAL_GetTick()-act_time_stamp) > freefire_trig_time )	//连发触发计时
		{ Freefire(); }
		else
		{
			if(once_flag==0)	//单次射击
			{
				Once(); 
				once_flag =1;
			}
			LocationRun();
		}
	}
	else
	{
		Angle_Set(softTargetAngle);//锁定电机在softTargetAngle
		once_flag = 0;//清空标志
		rammerStepLeft = 0;//ONCE的原理是增加步数。清空步数防止下次触发直接动
	}
	motor::CANSend();
}

/**
 * @brief  BURST模式管理函数
 * @param[in]  trigger 扳机标志，1为触发
 */

void AmmoFeed::BurstManage(uint8_t trigger)
{
	static uint8_t burst_flag =0;	//单次BURST标志位

	if( trigger )	//当扳机触发
	{
		if(burst_flag == 0)	//若标志位未立
		{	
			Burst();
			burst_flag = 1;	//立标志，开启BURST
		}
		else
		{;}
		 LocationRun();		//Burst在LocationRun模式下才能运行
	}
	else
	{
		burst_flag = 0;		//清空标志
		Angle_Set( softTargetAngle);	//锁定电机在softTargetAngle
		rammerStepLeft = 0;		//BURST的原理是增加步数。清空步数防止下次触发直接动
	}
	motor::CANSend();
}









#undef SIGN

