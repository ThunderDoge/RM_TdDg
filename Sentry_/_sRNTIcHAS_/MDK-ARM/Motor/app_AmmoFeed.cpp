#include "app_AmmoFeed.hpp"
#ifndef SIGN
#define SIGN(x) ((x)>0?1:((x)<0?-1:1))
#endif

void AmmoFeed::PR_Handle(void)
{
	static uint8_t last_mode;
if(!
		BlockedReaction())//堵转时运行堵转处理
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
		if( !IsBlockInHandle )	//堵转，未处理
		{	//正常堵转处理程序
			softTargetAngle = SoftAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//设定反转角度
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
			if( fabs(SoftAngle - softTargetAngle) <=3 )	
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

/** 
    * @brief 按步数运行的函数
*/
void AmmoFeed::PositionRun(void)
{
	static int16_t Step_Overflow;
	static uint32_t rammerStepTime = 0;
	if(( (HAL_GetTick() - rammerStepTime) > rammingDiscreDelay ) ) 
	{
		if(rammerStepLeft > 0)//如果存在步数
		{
			softTargetAngle += ( SIGN(ramDir) * 360.0f/feeder_division) ;		//设定反转角度，方向由用户设定的变量 ramDir 决定
			rammerStepLeft-- ;
			rammerStepTime = HAL_GetTick();
		}
		else if(rammerStepLeft == 0)//如果步数为0
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
* @brief  自由开火模式配置函数
* @param[in]   期望速度
* @retval  
* @par 日志 
*
*/
void AmmoFeed::Freefire_Set(int32_t FreeSpeed)
{
	AmmoFeed_mode = AMMOFEED_FREEFIRE;
	rammingContiSpeed = FreeSpeed;
}
/** 
* @brief  N连发模式配置函数
* @param[in]   DiscreDelay 每走一步间隔时间   trig N连发的触发条件 
* @retval   
* @par 日志 
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
* @brief  单步连发模式配置函数
* @param[in]   DiscreDelay 每走一步间隔时间   trig 连发的触发条件 
* @retval   
* @par 日志 
*
*/
void AmmoFeed::FreeOnce_Set(int32_t	DiscreDelay,int16_t* trig)
{
	AmmoFeed_mode = AMMOFEED_FREEONCE;
	rammingDiscreDelay = DiscreDelay;
	trigger = trig;
}

/** 
    * @brief 自由开火模式
*/
void AmmoFeed::Freefire(void)
{
	Speed_Set(rammingContiSpeed);
	softTargetAngle = RealAngle;
	motor::CANSend();
}


/** 
    * @brief 单步连发模式
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
			rammerStepLeft = 1;//单次发射，就给予一个步数
			once_flag = 0;
		}
		else
		{
			rammerStepLeft = 0;//不发射时把步数清零
		}
	}
	PositionRun();//按步数进行拨弹
}


/** 
    * @brief N连发模式
*/
void AmmoFeed::Burst(void)
{
	static uint8_t burst_flag =0;	//单次BURST标志位
	
	if( *trigger>200 )	//当扳机触发
	{
		if(burst_flag == 0)	//若标志位未立
		{	
			rammerStepLeft = BurstShootCnt;
			burst_flag = 1;	//立标志，开启BURST
		}
	}
	else
	{
		burst_flag = 0;
	}
	PositionRun();
}









#undef SIGN

