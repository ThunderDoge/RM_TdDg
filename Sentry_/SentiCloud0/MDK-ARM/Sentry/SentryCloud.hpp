/**
 * @file SentryCloud.hpp
 * @brief    哨兵上云台
 * @details     Encoding - UTF-8
 * @author   ThunderDoge
 * @date     2020-5-23
 * @version  v2.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 * 未特别标注的部分全由 ThunderDoge 编写
 * 
 * v1.0 2020-4-15   发布
 * v2.0 2020-5-23   重写了函数
 */
#ifndef __SENTRY_CLOUD_HPP_
#define __SENTRY_CLOUD_HPP_

// 依赖的文件
#include "sentry_ctrl_def.hpp"
#include "bsp_motor.hpp"
#include "app_vision.hpp"
#include "app_imu.h"
#include "app_mode.hpp"
#include "app_math.h"
#include "app_check.h"
#include "app_AmmoFeed.hpp"

// extern app_Mode ModeCloudCtrlMech;  // 机械角位置环，陀螺仪速度环
// extern app_Mode ModeCloudCtrlGyro;  // 陀螺仪 位置环&速度环

//extern app_Mode ModeDualPitch;
//extern app_Mode ModeSinglePitch;


typedef enum __sentry_cloud_dual_single_pitch_ctrl:uint8_t{
	__cloud_main_pitch=0U,
	__cloud_second_pitch=1U,
	__cloud_dual_pitch=2U,
}PitchModeEnum;

typedef enum __sentry_cloud_shoot_mode:uint8_t{
    
}ShootModeEnum;
typedef enum __sentry_cloud_shoot_trigger_source:uint8_t{

}ShootTrigger;

typedef enum __sentry_cloud_error_type:uint8_t{
    CLOUD_OK,
    SLOW_FRIC_BAN_FEED,
    MOTOR_LOST,
    CloudErrorCount,
}CloudError_t;


/**
 * @brief 云台物理实体类，包含 操纵云台进行物理运动的函数 和 物理信息，以及相关配置信息
 * 
 */
class SentryCloud   
{
public:
    //Initializer & Destructor 云台物理实体类 构造与删除函数
    SentryCloud(uint8_t yaw_can_num,uint16_t yaw_can_id, 
    uint8_t pitch_can_num, uint16_t pitch_can_id,
	uint8_t pitch2nd_can_num, uint16_t pitch2nd_can_id,
    uint8_t fric_l_can_num, uint16_t fric_l_can_id,
    uint8_t fric_r_can_num,uint16_t fric_r_can_id,
    uint8_t feed_can_num,uint16_t feed_can_id);

    /*--------------------------------成员声明------------------------------------------*/

	//请注意成员变量将以声明的顺序初始化

    /*------------------- 控制用紧急按钮变量，向这些变量写入1可以直接产生效果,DEBUG用 --------------------*/
    
    uint8_t RED_BUTTON_HaltAllPids;  ///< 停止所有PID输出


    /*---------------------------------实时更新物理信息----------------------------------*/

    // 操作这些变量不会产生控制效果

    // 陀螺仪数据经过旋转计算，储存在这里。周期性更新。
    float RotatedImuAngle[3];       ///<云台枪口朝向：Roll,Pitch,Yaw
    float RotatedImuAngleRate[3];   ///<云台枪口朝向角速度Roll,Pitch,Yaw
    float BaseImuAngleRate[3];      ///< 云台水平底座朝向

    // 云台运行状态
    CloudMode_t CloudMode;      ///<云台状态指示

    int32_t Shoot_Speed=4000;   ///<摩擦轮的设定速度。	
	
    float RealYaw;      ///< Yaw软角度
	float MechanicYaw;  ///< Yaw机械角度
    float RealPitch;    ///< Pitch软角度

    float TargetYaw;    ///< 受控制的时候设定的角度， DEBUG用
    float TargetPitch;  ///< 受控制的时候设定的角度， DEBUG用

    /*---------------------------------逻辑控制相关信息----------------------------------*/

    uint8_t MotorOnlineList[6];                     ///< 六个电机在线列表，依声明序
    uint32_t LastControlTick;                       ///< 标记最后受外界控制时间。便于观察视觉离线

    CloudError_t CloudError[CloudErrorCount];         ///<  错误信息

	PitchModeEnum pitch_ctrl_mode = __cloud_main_pitch;         ///< 云台单pitch/双pitch模式

    uint8_t pitch_exceed_flag[2];       ///< pitch超过软件限位的标志位
    uint8_t pitch_last_exceed_flag[2];  ///< pitch超过软件限位的标志位,上一次数值
    float pitch_IMax_save[2];           ///< pitch超过软件限位时关闭IMAX，IMAX保存到这里

    /*-------------------------------------电机用PID---------------------------------------*/

    // 你可以直接操作电机与其PID来直接产生控制效果

	pid PitchSpeed;         ///<Pitch电机机械角 速度环
    pid PitchPosition;      ///<Pitch电机机械角 位置环
    pid PitchGyroSpeed;     ///<Pitch电机陀螺仪 速度环
    pid PitchGyroPosition;  ///<Pitch电机陀螺仪 位置环
	
	pid Pitch2ndSpeed;         ///<Pitch电机机械角 速度环
    pid Pitch2ndPosition;      ///<Pitch电机机械角 位置环
    pid Pitch2ndGyroSpeed;     ///<Pitch电机陀螺仪 速度环
    pid Pitch2ndGyroPosition;  ///<Pitch电机陀螺仪 位置环

    pid DualSpeed;           ///<Dual电机机械角 速度环
    pid DualPosition;        ///<Dual电机机械角 位置环
    pid DualGyroSpeed;       ///<Dual电机陀螺仪 速度环
    pid DualGyroPosition;    ///<Dual电机陀螺仪 位置环

    pid YawSpeed;           ///<Yaw电机机械角 速度环
    pid YawPosition;        ///<Yaw电机机械角 位置环
    pid YawGyroSpeed;       ///<Yaw电机陀螺仪 速度环
    pid YawGyroPosition;    ///<Yaw电机陀螺仪 位置环

    pid FricLeftSpeed;      ///<左边摩擦轮 速度环
    pid FricRightSpeed;     ///<右边摩擦轮 速度环

    pid FeedSpeed;          ///<供弹轮 速度环
    pid FeedPositon;        ///<供弹轮 位置环

    /*----------------------------------------电机---------------------------------------*/

    // 你可以直接操作电机与其PID来直接产生控制效果

    softcloud YawMotor;     ///<Yaw轴电机对象_CAN1上
    softcloud PitchMotor;   ///<Pitch轴电机对象_CAN1上
	softcloud PitchSecondMotor;		///<从动Pitch轴电机对象_CAN2上

    // 警告⚠ 当摩擦轮速度低于一值时，不允许开启拨弹电机。由Handle()强制修正，直接控制Feed2nd时无效的。

    motor FricLeftMotor;    ///< 左边摩擦轮
    motor FricRightMotor;   ///< 右边摩擦轮
    AmmoFeed Feed2nd;       ///< 供弹轮电机

	/*------------------------- 控制用函数，使用这些函数会产生控制效果 --------------------------*/

    void Handle();      ///< 【重要】云台自动控制函数，包含着所有数据的获取、处理和例行执行。应当在主逻辑任务中调用。

    void Safe_Set();                                ///< 安全模式
    void SetAngleTo(float pitch, float yaw);        ///< 机械角度设定
    void SetAngleTo_Gyro(float pitch, float yaw);   ///< 角度设定 陀螺仪控制模式
    void SetCloudMode(CloudMode_t newCloudMode);    ///< 设定云台模式

	void Shoot(float bullet_speed, uint8_t fire_freq, ShootModeEnum shoot_mode);  ///< 设定射击模式

	void LazerSwitchCmd(int OnOrOff);   ///<开关激光灯
    void ShooterSwitchCmd(int OnOrOff); ///<开关摩擦轮
	void SetPitchRealAngleLimit(float max, float min);  ///< 设定软件限位角度，参数的标准同PitchMotor.RealAngle;
	
private:    

    /* -------------------------------私有变量，仅内部用------------------------------- */
	
    static const float RotationMatrix[3][3];    ///<旋转矩阵陀螺仪到云台枪口方向。现在没用 
    
	float pitch_limit_max;  ///< 云台软限位上限
	float pitch_limit_min;  ///< 云台软限位上限

    PitchModeEnum last_pitch_ctrl_mode = __cloud_main_pitch;    ///< 云台单pitch/双pitch模式，历史
	
	/*----------------------------------- 内部运行函数 -----------------------------------*/

    // 后缀Ctrl的函数会控制电机产生控制效果
    // 后缀Handle的函数仅控制信息显示，如上面的控制变量
	
	
	void PitchRealAngleLimitCtrl(void); ///< 软件限位
	void PitchModeCtrl(void);           ///< 双PITCH/单PITCH切换逻辑
    void ShootCtrl(void);               ///< 射击控制逻辑执行函数，在Handle中调用
    void RedButtonEffectCtrl(void);     ///< 实现紧急开关的功能
	void ImuDataProcessHandle(void);    ///< 陀螺仪数据处理
    void ErrorReportHandle(void);       ///< 错误信息输出
	
    //废案 5-23
	// void EnterModeDualPitch(void);      
	// void RunModeDualPitch(void);
	// void ExitModeDualPitch(void);
};

extern SentryCloud CloudEntity; ///云台物理实体对象。包含电机激光器等设备。

//云台控制模式相关
// void EnterModeCloudCtrlMech(void);
// void RunModeCloudCtrlMech(void);
// void EnterModeCloudCtrlGyro(void);
// void RunModeCloudCtrlGyro(void);
// extern app_Mode ModeCloudCtrlMech;  // 机械角位置环，陀螺仪速度环
// extern app_Mode ModeCloudCtrlGyro;  // 陀螺仪 位置环&速度环

// //// 双PITCH控制相关
// //void PitchModeCtrl(void);


// //extern app_Mode ModeDualPitch;
// //extern app_Mode ModeSinglePitch;
// //extern app_Mode ModeSecondPitch;

// /// PID运算回调。用于PITCH重力前馈 逻辑
// void pidPitchCallBack(pid* self);

#endif // __SENTRY_CLOUD_HPP_
