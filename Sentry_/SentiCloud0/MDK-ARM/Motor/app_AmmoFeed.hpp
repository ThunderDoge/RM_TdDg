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
        }//!<构造函数
		AMMOFEED_MODE feed_mode;
		
		int32_t rammingContiSpeed = 0;		//速度控制的运行速度
		int32_t	rammingDiscreDelay = 0;		//路程控制，每个隔得停顿时间
		uint8_t feeder_division = 6;		//拨弹有几个格
		uint32_t rammerStepTime = 0;		
		int32_t rammerStepLeft = 0;			//拨弹剩余格数。路程控制会消耗这个计数。
		float softTargetAngle = 0;			//软路程角度设定值
		float rev_angle_when_blocked = 30;	//堵转时回转的角度
		int8_t ramDir = 1;					//位置控制，指示转动方向。正负随电机编码器增长方向。
		uint16_t freefire_trig_time = 150;	//按住切到连发切换延时
		uint8_t BurstShootCnt = 3;			//N连发的N

		uint8_t BlockedReaction(void);	//堵转反应处理过程
		
		void SpeedRun(void);			//速度控制模式
		void LocationRun(void);			//位置控制模式
		
		void Freefire(void);			//连发模式
		void Burst(void);				//N连发模式，N=BurstShootCnt
		void Once(void);				//单发模式
		void Stop(void);				//停止
		
		void FreeAndOnceManage(uint8_t trigger);	//按下单发，按住连发
		void BurstManage(uint8_t trigger);			//N连发控制
	private:
		uint8_t IsBlockInHandle=0;
};


#endif	//APP_RAMMER_H_
