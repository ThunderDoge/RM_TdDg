/**
  * @file       app_AmmoFeed.hpp
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
 * @brief AmmoFeed类运行模式 枚举量
 * 
 */
enum ammofeed_status_enum {
	AMMOFEED_FREE_FIRE,
	AMMOFEED_BURST,
	AMMOFEED_FREE_ONCE,
};


class AmmoFeed : public softmotor
{
    friend class SentryCloud;   ///使得SentryCloud可以调用供弹控制函数
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
					 feeder_division(RamerDiv) , rammer_direction(rammer_direction){}///<构造函数				
						 

						 
			int32_t ramming_speed = 0;		///<PID速度环的目标速度。
						 
			int32_t	ramming_discrete_delay = 10;		//路程控制时，每个隔的停顿时间 注：N连发模式下间隔时间需要较长
					// 停顿时间值"2"可用 12-2-16：13-Eno
			uint8_t feeder_division = 7;		//拨弹轮有几个格
					 
			int32_t rammer_step_left = 0;			//拨弹剩余步数。路程控制会消耗这个计数。
										 
			float soft_target_angle = 0;			//软路程角度设定值
					
						 
			float rev_angle_when_blocked = 20;	//堵转时回转的角度,默认20度
						 
			int8_t rammer_direction = 1;					//位置控制，指示转动方向。正负随电机编码器增长方向。
			
		    uint8_t burst_shoot_cnt = 3;			//N连发的N

			uint8_t Blocked_Reaction(void);	//堵转反应处理过程
			
			virtual void Step_Run(void);			//位置控制模式
		
			virtual void  Safe_Set(void) override;
			
			int16_t trigger;//Free_Once和Burst的触发条件
			uint8_t feed_mode;		//拨弹模式指示
			uint16_t free_once_trig_time = 150;	//按住切到单步连发切换的延时时间
			
		protected:	
			void Free_Fire_Set(int32_t FreeSpeed);			//流畅运转模式配置
			void Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig);				//N连发模式配置
			void Free_Once_Set(int32_t	DiscreDelay,int16_t trig);				//单发模式配置
			virtual void Handle(void);
	
			void Free_Fire(void);			//流畅运转模式
			void Burst(void);				//N连发模式，N=burst_shoot_cnt
			void Free_Once(void);		    //连续单发
            //内部状态变量
			uint8_t is_block_in_handle = 0;     //用于反转堵转检测
		
};


#endif
