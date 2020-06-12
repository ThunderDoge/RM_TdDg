/**
* @file     app_AmmoFeed.hpp
* @brief    通用拨弹电机库源文件
* @details  
* @author    ThunderDoge, Asn
* @date      2020.5
* @version  1.0.9
* @copyright  RM2020电控 
* @par 日志
*		v1.0.0  2019/11/29  实现基本功能\n
*		v1.0.1  2019/12/6   Asn进行了部分简化，使其易于使用\n
*		v1.0.2  2019/12/13  修正了部分多余的依赖，以及一些语法错误。拨弹回转处理部分Blocked_Reaction仍存在死锁风险，可以PID参数解决\n
*		v1.0.3  2019/12.27  修正堵转判断逻辑，并统一用外部cansend函数，然后更新命名，使其更规范\n
*		v1.0.4  2019/12/31  把原来的Pr_Handle函数移植到Handle中，使Set完模式之后只需要调用manager::CANSend()就好了\n
*		v1.0.5  2020/1/21   修正安全模式的bug，并增加专用停止模式，并增加trig宏定义\n
*		v1.0.6	2020/2/24	增加：在每次trig触发后对触发值清0，Set_Step函数可在外部直接设置步数
*       v1.0.7  2020/5/22   修正 last_mode错误地使用static使得多个ammofeed对象互相干扰的问题 
*		v1.0.8  2020/5/26	修正free_once在转换模式中可能出现的问题
*       v1.0.9  2020/5/29   去除了所有函数内使用的的static变量，变为对象private变量
*/ 
#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

#define DBUS_RANGE    0 //1，dbus通道值范围解算出来是364~1684
						//0，dbus通道值范围解算出来是0~660


/** 
* @brief 周期执行的函数
*/
void AmmoFeed::Handle(void)
{
	if(feed_mode != last_feed_mode)
	{
		soft_target_angle = SoftAngle;//模式切换时角度同步
	}
	if(block!=NULL && RunState!=Stop)
	{
		block->Block_Check();
		if(!Blocked_Reaction())//堵转时运行堵转处理
		{
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
	}
	last_feed_mode = feed_mode;
}


/**
* @brief 堵转处理函数
* @retval 0 正常退出
* @retval 其他 异常退出
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
* @brief 自由开火模式配置函数
* @param FreeSpeed 期望速度
*/
void AmmoFeed::Free_Fire_Set(int32_t FreeSpeed)
{
	feed_mode = AMMOFEED_FREE_FIRE;
	RunState = Speed_Ctl;
  TargetSpeed = FreeSpeed;
}

/**
* @brief N连发模式配置函数
* @param DiscreDelay 每走一步间隔时间
* @param trig N连发的触发条件
*/
void AmmoFeed::Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig)
{
	feed_mode = AMMOFEED_BURST;
	RunState = Position_Ctl;//只用做安全模式切换和指示作用
	burst_shoot_cnt = ShootCnt;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}

/**
* @brief 单步连发模式配置函数
* @param DiscreDelay 每走一步间隔时间
* @param trig 连发的触发条件 
*/
void AmmoFeed::Free_Once_Set(int32_t DiscreDelay,int16_t* trig)
{
	feed_mode = AMMOFEED_FREE_ONCE;
	RunState = Position_Ctl;//只用做安全模式切换和指示作用
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/**
* @brief 目标角度设置函数
* @details 和softmotor一样，只是去掉了RunState设置，因为RunState设置被放在了拨弹的用户接口处
*/
void AmmoFeed::Angle_Set(float Target_Angle)
{
	if(Target_Angle>max)Target_Angle=max;//对目标角度限幅
	if(Target_Angle<min)Target_Angle=min;
	Target_Angle=Target_Angle*MotorType->Reduction_ratio/360;//得到电机减速前目标圈数
	TargetPosition=(Target_Angle-(int32_t)Target_Angle)*MotorType->max_mechanical_position;//小数部分 减速前单圈位置
	Soft_TargetPosition=(int32_t)Target_Angle;//整数部分 即减速前圈数
}
/**
 * @brief 拨弹电机专用安全模式
 * 
 */
void AmmoFeed::Safe_Set()
{
	if(block!=NULL)block->IsBlock=0;//去除堵转标志，避免在逻辑中依然认为是堵转
	feed_mode = AMMOFEED_STOP;
	RunState = Stop;
	rammer_step_left = 0;
	soft_target_angle = SoftAngle;
	Angle_Set(soft_target_angle);
    softmotor::Safe_Set();
}
/**
* @brief 专用停止模式
*/
void AmmoFeed::Stop_Set(void)
{
	if(block!=NULL)block->IsBlock=0;//去除堵转标志，避免在逻辑中依然认为是堵转
	feed_mode = AMMOFEED_STOP;
	RunState = Stop;
	rammer_step_left = 0;
	soft_target_angle = SoftAngle;
	Angle_Set(soft_target_angle);
}

/*************************************************************************/



/****************************模式运行函数*********************************/
/** 
* @brief 自由开火模式
*/
void AmmoFeed::Free_Fire(void)
{
	soft_target_angle = SoftAngle;//此处为冗余，防止模式切换反转
	Speed_Run();
}


/** 
* @brief 单步连发模式
*/
void AmmoFeed::Free_Once(void)
{
	if(last_feed_mode != feed_mode)
	{
		activated_flag = 0;
		once_flag = 0;
	}
	if( *trigger>trig_set )
	{
		if( activated_flag == 0 )
		{
			activated_flag= 1;
			act_time_stamp = HAL_GetTick();//用于判断单发还是连发
		}
		*trigger = 0;		//清除触发
	}
	else
	{	
		activated_flag = 0;	
	}
	if(activated_flag)
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
    if(last_feed_mode != feed_mode)
	{
		activated_flag = 0;
	}

	if( *trigger>trig_set )	//当扳机触发
	{
		if(activated_flag == 0)	//若标志位未立
		{	
			rammer_step_left = burst_shoot_cnt;
			activated_flag = 1;	//立标志，开启burst
		}
		*trigger = 0;		//清除触发
	}
	else
	{
		activated_flag = 0;
	}
	Step_Run();
}


/*****************************************************************************/




#undef SIGN
