/**
 * @file        SentryChassis.hpp
 * @brief       哨兵底盘物理实体
 * @details     
 *  哨兵底盘主要功能硬件有：
 *  - 底盘主动轮电机，板上有功率检测芯片检测它的实时电流与功率
 *  - 与底盘固定的陀螺仪
 *  - 沿轨道左右各一：GY-53/VL531LX廉价激光测距模块。测距范围2000mm，精度在10mm
 *  - 沿轨道左右各一：弹簧。允许以不超过X.X mm/s的速度撞击轨道，并保护底盘主体不受撞击
 *      - 上下云台归云台自己控制，底盘不控制。
 *      - 底盘
 *  需要实现以下功能：
 *  - 根据现有传感器尽可能精确估计轨上位置
 *  - 底盘主动轮控制。含功率控制
 * 
 * @author   ThunderDoge
 * @date      2020-4-11
 * @version   v1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 * 注：在2019-12月之前的哨兵底盘具有不同的结构：
 * 弹药舱装在底盘，底盘有两个拨弹轮分别向上下拨弹；另有一摩擦轮辅助向上云台供弹管 推弹丸。
 * 在2019-1月之前被废弃。代码上可能有残留部分，但大部分已经注释。
 *  verison|    date|       author|         change|
 *  1.0          2020-4-11   ThunderDoge     添加了更详细的注释
 */
#ifndef __SENTRY_CHASSIS_HPP_
#define __SENTRY_CHASSIS_HPP_

// #include "app_AmmoFeed.hpp"  // 2020-1 之后不再需要拨弹
#include "bsp_stddef.h"
#include "app_imu.h"
#include "bsp_motor.hpp"
#include "bsp_current.h"
//#include "bsp_adc_deal.h"
#include "bsp_encoder.hpp"
#include "bsp_gy53l1.h"
#include "app_math.h"
#include <cmath>
#include <cstring>
#include "app_mode.hpp"
#include "sentry_ctrl_def.hpp"

// 测距模块连接的串口
#define RANGING_LEFT_UART	huart3
#define RANGING_RIGHT_UART	huart4










extern float SpeedMax;

/**
 * @brief 与柱子距离的枚举量
 */
enum PillarFlagEnum : int8_t
{
    PILLAR_NOT_DETECTED = 0x00,    ///< 未检测到柱子
    PILLAR_LEFT = 0X01,         ///< 检测到左侧柱子
	PILLAR_BOUNCE_LEFT=0x02,    ///< 撞击左侧柱子
    PILLAR_RIGHT = 0X03,        ///< 检测到右侧柱子
	PILLAR_BOUNCE_RIGHT=0x04,   ///< 撞击右侧柱子
};

/**
 * @brief 底盘物理实体类
 */
class SentryChassis
{
    friend class manager;   // 使得电机可以访问哨兵内部变量

public:
    SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id);    ///<    构造函数
//                  uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
//                  uint8_t up_feed_can_num, uint16_t up_feed_can_id,
//                  uint8_t down_feed_can_num, uint16_t down_feed_can_id,
//                  uint8_t up_fric_can_num, uint16_t up_fric_can_id);
    //------------------------系统变量
    static SentryChassis* pointer;
    //-------------------------PID变量
    pid pidDriveSpeed;      ///< 底盘主动轮速度环
    pid pidDriveLocation;   ///< 底盘主动轮位置环
    // pid FricSpeed;
    // pid FricLocation;
    // pid FeedUpSpeed;
    // pid FeedUpLocation;
    // pid FeedDownSpeed;
    // pid FeedDownLocation;
    pid pidDriveCurrent;    ///< 底盘主动轮功率控制电流环节
    pid pidPowerFeedback;   ///< 底盘主动轮功率控制电流反馈环节
    //-------------------------电机变量
    softmotor DriveWheel;
//    motor Fric;
//    AmmoFeed FeedUp;
//    AmmoFeed FeedDown;
	//-------------------------激光测距GY-53/VL53L1X
	bsp_GY53L1_Object RangingLeft;
	bsp_GY53L1_Object RangingRight;

    //-------------------------运行状态参数
    _chassis_mode Mode;
    _chassis_mode LastMode;
    //-------------------------物理状态参数，可供查询
    float MotorSpeed;           ///< 电机反馈速度，单位rpm
    float MotorSoftLocation;    ///< 电机软路程，单位是角度
    float RealSpeed;            ///< 真正的速度，单位mm/s；由编码器
    float RealPosition;         ///< 由编码器与激光测距模块 数据融合估计的真正的距离，单位mm
	float Accel_Railward;	    ///<  沿轨道的加速度
	int16_t LazerRanging[2];	///< 激光测距得出的距离，单位mm
	float imuLeftBounceThreshold=5;		///<  判定为撞击立柱的陀螺仪加速度阈值
	enum PillarFlagEnum PillarFlag = PILLAR_NOT_DETECTED;
	//-------------------------功率计算参数

    float DrivePower;      ///< 驱动轮功率，单位W. 供查询
    float LimitPower = -1; ///<  底盘限制功率, <0 表示不限制
    //-------------------------运行托管函数
    void Handle();                                         ///< 运行时数据更新和逻辑功能 函数
    //-------------------------操作函数
    void Safe_Set();                                       ///< 安全模式
    void MotorSpeed_Set(float speed_motor_rpm);            ///< 设定电机速度
    void MotorSoftLocation_Set(float location_motor_soft); ///< 设定电机软路程
    void MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit);  ///< 设定电机软路程，限速运行
    // void Speed_Set(float speedBy_mm_s); //设定真实速度
    // void Location_Set(float locationBy_mm); //设定真实位置。
private:
    //功率计算用变量
    float TargetPowerInput = 0; 
    float PowerOutput = 0;
    uint32_t PwrUpdateTime = 0;
    LPF2 lpf;


    void CanSendHandle(); //托管到CANSend的操作函数
    float PowerFeedbackSystem(float TargetSpeedInput, float TargetCurInput,float PwrFeedbackInput);//柴小龙式功率闭环

    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
};
extern SentryChassis ChassisEntity;     // 底盘实体对象

#endif // __SENTRY_CHASSIS_HPP_
