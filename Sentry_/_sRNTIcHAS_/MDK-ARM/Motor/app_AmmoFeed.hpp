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
					 feeder_division(RamerDiv) , ramDir(ramDir){}//构造函数				
						 

						 
			int32_t rammingContiSpeed = 0;		//速度控制的运行速度
						 
			int32_t	rammingDiscreDelay = 10;		//路程控制，每个隔得停顿时间 注：N连发模式下间隔时间需要较长
					// 停顿时间值"2"可用 12-2-16：13-Eno
			uint8_t feeder_division = 7;		//拨弹有几个格
					 
			int32_t rammerStepLeft = 0;			//拨弹剩余步数。路程控制会消耗这个计数。
										 
			float softTargetAngle = 0;			//软路程角度设定值
					
						 
			float rev_angle_when_blocked = 30;	//堵转时回转的角度,默认30度
			uint8_t IsBlockInHandle = 0;//用于反转堵转检测
						 
			int8_t ramDir = 1;					//位置控制，指示转动方向。正负随电机编码器增长方向。
			
		  uint8_t BurstShootCnt = 3;			//N连发的N

			uint8_t BlockedReaction(void);	//堵转反应处理过程
			
			void PR_Handle(void); 			//周期执行函数
			virtual void PositionRun(void);			//位置控制模式
			
			void Freefire_Set(int32_t FreeSpeed);			//流畅运转模式配置
			void Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig);				//N连发模式配置
			void FreeOnce_Set(int32_t	DiscreDelay,int16_t* trig);				//单发模式配置
			int16_t* trigger;
			
		protected:		
			void Freefire(void);			//流畅运转模式
			void Burst(void);				//N连发模式，N=BurstShootCnt
			void FreeOnce(void);		//连续单发
		private:
			uint8_t AmmoFeed_mode;		//拨弹模式指示
		
			uint16_t freeonce_trig_time = 150;	//按住切到单步连发切换的延时时间
};


#endif	//APP_RAMMER_H_
