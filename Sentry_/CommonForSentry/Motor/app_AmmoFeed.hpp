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
#ifndef __APP_AMMOFEED_H
#define __APP_AMMOFEED_H

#include "math.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include "bsp_dbus.h"

//拨弹模式枚举
typedef enum
{
    AMMOFEED_FREE_FIRE, //!<自由射击模式，不按格数拨弹
    AMMOFEED_BURST,     //!<N连发模式，按格数发射
    AMMOFEED_FREE_ONCE, //!<按格数发射模式
    AMMOFEED_STOP       //!<停止模式，非安全模式
} FeedModeEnum;

///拨单电机库类
class AmmoFeed : public softmotor
{
public:
    AmmoFeed(uint8_t can_num,
             uint16_t _can_id,
             Motor_t *motor_type,
             uint8_t RamerDiv,
             int8_t rammer_direction,
             pid *PID_In,
             pid *PID_Out = NULL)
        : softmotor(can_num, _can_id, motor_type, PID_In, PID_Out),
          feeder_division(RamerDiv), rammer_direction(rammer_direction) {} //构造函数

    AmmoFeed(): softmotor() {}  // 空对象的构造函数

    void Set_Step(int32_t rammer_step) //设置步数，给想在外部直接设定步数提供接口
    {
        rammer_step_left = rammer_step;
    }
	
	int32_t get_rammer_step_left (void){return rammer_step_left;}

    uint8_t Blocked_Reaction(void); //堵转反应处理过程

    virtual void Step_Run(void);                //位置控制模式
    virtual void Angle_Set(float Target_Angle); //角度设置

    void Free_Fire_Set(int32_t FreeSpeed);                                //流畅运转模式配置
    void Burst_Set(uint8_t ShootCnt, int32_t DiscreDelay, int16_t *trig); //N连发模式配置
    void Free_Once_Set(int32_t DiscreDelay, int16_t *trig);               //单发模式配置
    void Stop_Set(void);                                                  //停止模式，非安全
    virtual void Safe_Set(void) override;

protected:
    virtual void Handle(void);

    void Free_Fire(void); //流畅运转模式
    void Burst(void);     //N连发模式，N=burst_shoot_cnt
    void Free_Once(void); //连续单发

private:
    int32_t ramming_discrete_delay = 10; //路程控制，每个隔得停顿时间 注：N连发模式下间隔时间需要较长

    uint8_t feeder_division = 7; //拨弹有几个格

    int32_t rammer_step_left = 0; //拨弹剩余步数。路程控制会消耗这个计数。

    float soft_target_angle = 0; //软路程角度设定值

    float rev_angle_when_blocked = 30; //堵转时回转的角度,默认30度

    uint8_t is_block_in_handle = 0; //用于反转堵转检测

    int8_t rammer_direction = 1; //位置控制，指示转动方向。正负随电机编码器增长方向。

    uint8_t burst_shoot_cnt = 3; //N连发的N，默认三连发

    FeedModeEnum feed_mode; //拨弹模式指示

    FeedModeEnum last_feed_mode;

    int16_t *trigger; //Free_Once和Burst的触发条件

    uint16_t free_once_trig_time = 150; //按住切到单步连发切换的延时时间
    int16_t Step_Overflow;
    uint32_t rammerStepTime;
    uint8_t act_flag;
    uint32_t act_time_stamp;
    uint8_t once_flag;
    int16_t trig_set = 200;
    uint8_t activated_flag;
};

#endif
