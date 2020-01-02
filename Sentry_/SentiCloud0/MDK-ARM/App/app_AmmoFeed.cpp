/**
  * @brief    通用拨弹电机库
  * @details  
  * @author   ThunderDoge & Asn
  * @date     2019/12/13
  * @version  v1.0.3 
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  * v1.0.0  2019/11/29  实现基本功能
  * v1.0.1  2019/12/6   Asn进行了部分简化，使其易于使用。
  * v1.0.2  2019/12/13  修正了部分多余的依赖，以及一些语法错误。拨弹回转处理部分Blocked_Reaction仍存在死锁风险，可以PID参数解决。
	*	v1.0.3  2019/12.27  修正堵转判断逻辑，并统一用外部cansend函数，然后更新命名，使其更规范
	* v1.0.4  2019/12/31  把原来的Pr_Handle函数移植到Handle中，使Set完模式之后只需要调用manager::CANSend()就好了
  */

#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

/** 
    * @brief 周期执行的函数
*/
void AmmoFeed::Handle(void)
{
	static uint8_t last_mode;
	if(block!=NULL && RunState!=Stop)
	{
		block->Block_Check();
	}
	if(!Blocked_Reaction())//堵转时运行堵转处理
	{
		if(feed_mode != last_mode)
		{
			soft_target_angle = SoftAngle;
		}
		switch(feed_mode)
		{
			case AMMOFEED_FREE_FIRE:
				Free_Fire();
				break;
			case AMMOFEED_BURST:
				Burst();
				break;
			case AMMOFEED_FREE_ONCE:
				Free_Once();
				break;
			default:
				Safe_Set();
				break;
		}
	}
	last_mode = feed_mode;
}


/** 
* @brief  堵转处理函数
* @param[in]   无 
* @retval   是否在处理堵住的标志位 1代表在处理，0表示没有开启堵转或者没有堵转
* @par 日志 
*
*/
uint8_t AmmoFeed::Blocked_Reaction(void)
{
	if(block == NULL)
		return 0;
	if(block->IsBlock)
	{
		if( !is_block_in_handle )	//堵转，未处理
		{	//正常堵转处理程序
			soft_target_angle = SoftAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//设定反转角度
			Angle_Set(soft_target_angle);
			block->Clear_BlockFlag();	//清楚堵转标志
			is_block_in_handle = 1;	//立起处理标志
		}
		else //在处理，仍堵转
		{
			is_block_in_handle = 0;	//强制回到正常
			block->Clear_BlockFlag();
		}
	}
	else
	{
		if(is_block_in_handle)	//未堵转，在处理。
		{
			if( fabs(SoftAngle - soft_target_angle) <=3 )	
			{
				is_block_in_handle = 0;	//达到角度后返回正常
			}
			else
			{
				Angle_Set(soft_target_angle);	//继续处理
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
void AmmoFeed::Step_Run(void)
{
	static int16_t Step_Overflow;
	static uint32_t rammerStepTime = 0;
	if(( (HAL_GetTick() - rammerStepTime) > ramming_discrete_delay ) ) 
	{
		if(rammer_step_left > 0)//如果存在步数
		{
			soft_target_angle += ( SIGN(rammer_direction) * 360.0f/feeder_division) ;		//设定反转角度，方向由用户设定的变量 rammer_direction 决定
			rammer_step_left-- ;
			rammerStepTime = HAL_GetTick();
		}
		else if(rammer_step_left == 0)//如果步数为0，运行到附近的按格数计算的角度
		{
			Step_Overflow = (int16_t)floorf((soft_target_angle - SoftAngle)/(360.0f/feeder_division)/SIGN(rammer_direction));
			if(Step_Overflow<=0)Step_Overflow = 0;
			soft_target_angle -= ( SIGN(rammer_direction) * 360.0f/feeder_division)*Step_Overflow;
		}
	}
	Angle_Set(soft_target_angle);
	Position_Run();
}



/****************************模式配置函数*********************************/
/** 
* @brief  自由开火模式配置函数
* @param[in]   期望速度
* @retval  
* @par 日志 
*
*/
void AmmoFeed::Free_Fire_Set(int32_t FreeSpeed)
{
	feed_mode = AMMOFEED_FREE_FIRE;
	ramming_speed = FreeSpeed;
}
/** 
* @brief  N连发模式配置函数
* @param[in]   DiscreDelay 每走一步间隔时间   trig N连发的触发条件 
* @retval   
* @par 日志 
*
*/
void AmmoFeed::Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig)
{
	feed_mode = AMMOFEED_BURST;
	burst_shoot_cnt = ShootCnt;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/** 
* @brief  单步连发模式配置函数
* @param[in]   DiscreDelay 每走一步间隔时间   trig 连发的触发条件 
* @retval   
* @par 日志 
*
*/
void AmmoFeed::Free_Once_Set(int32_t	DiscreDelay,int16_t trig)
{
	feed_mode = AMMOFEED_FREE_ONCE;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/*************************************************************************/



/****************************模式运行函数*********************************/
/** 
    * @brief 自由开火模式
*/
void AmmoFeed::Free_Fire(void)
{
	Speed_Set(ramming_speed);
	soft_target_angle = RealAngle;
	Speed_Run();
}


/** 
    * @brief 单步连发模式
*/
void AmmoFeed::Free_Once(void)
{
	static uint8_t act_flag=0;
	static uint32_t act_time_stamp;
	static uint8_t once_flag = 0;

	if( trigger )
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
		if((HAL_GetTick()-act_time_stamp)>free_once_trig_time)//到达触发时间
		{
			rammer_step_left++;			
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
			rammer_step_left = 1;//单次发射，就给予一个步数
			once_flag = 0;
		}
		else
		{
			rammer_step_left = 0;//不发射时把步数清零
		}
	}
	Step_Run();//按步数进行拨弹
}


/** 
    * @brief N连发模式
*/
void AmmoFeed::Burst(void)
{
	static uint8_t burst_flag =0;	//单次burst标志位
	
	if( trigger )	//当扳机触发
	{
		if(burst_flag == 0)	//若标志位未立
		{	
			rammer_step_left = burst_shoot_cnt;
			burst_flag = 1;	//立标志，开启burst
		}
	}
	else
	{
		burst_flag = 0;
	}
	Step_Run();
}


void AmmoFeed::Safe_Set(void){
	Speed_Set(0);
	rammer_step_left = 0;
	feed_mode = 0;
}
/*****************************************************************************/








#undef SIGN

