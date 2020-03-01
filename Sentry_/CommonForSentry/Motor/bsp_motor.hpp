/** 
  * @file         bsp_motor.hpp 
  * @author      WMD,Onion rain
  *             Evan-GH
  *             ThunderDoge
  * @version  7.7b
  * @par Copyright (c):  
  *       WMD,Onion rain
  *				Evan-GH
  * @par 日志见cpp
  */
#ifndef __BSP_MOTOR_HPP
#define __BSP_MOTOR_HPP

#include "sentry_config.hpp"

#include "bsp_car_config.hpp"
using namespace std;

typedef struct str_Motor_t
{
    str_Motor_t(){}; //避免创建�????象不�????
    str_Motor_t(uint16_t a, uint8_t b) : max_mechanical_position(a), Reduction_ratio(b){};
    uint16_t max_mechanical_position = 8192; //!<�???????最�???,�????0x2000
    uint8_t Reduction_ratio = 19;            //!<减速比,�????19:1
} Motor_t;                                   //!<电机(电调)参数结构体，�????值为3508电机及其�????C620电调

class pid; //先声明一�? by thunderdoge 2020-2-27
/**
 * @brief 用于配置PID的用户自定义回调函数的指�?
 */
typedef void (*pvPidCallBack)(pid*);
//PID???以算pid输出 ???以配置pid
class pid
{
public:
    float *Custom_Diff = NULL; //!<???定义�????�????点型???分数??? 常用于路程环的微分环???(即速度???)
    pvPidCallBack pid_run_CallBack=nullptr; 
    uint16_t I_Time;           //!<pid时间参数 ???ms为单??? plus专属,???分时???
    uint16_t D_Time;           //!<???分时间，ms为单???
    uint16_t I_Limited;        //!<�???????小于I_Limited时才�????I输出 plus专属
    float ap = 0, bp = 0, cp;  //ap==0 && bp==0说明不是非线???pid
    float ai = 0, ci;
    float ad = 0, bd = 0, cd, dd;
    float P;
    float I;
    float D;
    float IMax;
    float PIDMax;

    //???共函???
    pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time = 1, uint16_t D_Time = 1, uint16_t I_Limited = 9999); //传统pid构造函???
    pid(float ap, float bp, float cp,
        float ai, float ci,
        float ad, float bd, float cd, float dd,
        float IMax, float PIDMax, uint16_t I_Time = 1, uint16_t D_Time = 1, uint16_t I_Limited = 9999); //非线???pid构造函???
    float pid_run(float err);
    float nonlinear_pid_run(float err);
    float sech(float in);

//private:
    //运算储存???
    float Pout;
    float Iout;
    float Dout;
    float Dout_Accumulative; //因为有微分时间重定义,�????加入一???变量�????用于输出D,源Dout用作????????????
    float PIDout;
    float CurrentError;
    float LastError;
    uint32_t I_start_time; //!<???分开始时间戳，用于带时间参数的pid   plus专属
    uint32_t D_start_time; //!<???分开始时间戳，用于带时间参数的pid
};

/** 
	* @brief 堵转检测类 
	* @details 要求在初始化的时候给定需�????测堵??的电流引用和位置引用 \n
	*		  一???不会单独使用该类，电�????�????持堵???检测功能的话会在其内部有一???堵转检测的指针 \n
	*		  �????�?用电机的堵转检测功能的话使??????应电机类内提供的初�?�化函数 \n
	*		  具体检测还需要提供堵???认定电流，堵???认定时间，堵???时间内允许的电机角度偏移??? \n
	*		  ???以通过 @see block_type::IsBlock 变量来确认电机是否堵??? \n 
	*		  ???以通过 @see block_type::Clear_BlockFlag() 来清除堵???标志
*/
class block_type
{
public:
    void Block_Init(uint16_t Limit, uint16_t time, float err_num);                                 //!<�????�????参数�????�?
    block_type(int16_t &_Current, float &_RealAngle) : Current(_Current), RealAngle(_RealAngle) {} //!构造函???
    void Block_Check(void);                                                                        //!<检查堵???的函???
    void Clear_BlockFlag(void);                                                                    //!<去除堵转标志
    uint8_t IsBlock;                                                                               //!<???否堵???的标志，1为已经堵???，需要人为使用Clear_BlockFlag消堵???标志
private:
    int16_t &Current;
    float &RealAngle;
    float block_Angle;            //!<堵转时的位置
    uint16_t block_Current_Limit; //!<堵转检测最低门限电???
    uint32_t block_time;          //!<堵转开始时的时间戳
    uint32_t block_Time_Limit;    //!<堵转检测时�????的堵�?时间
    float block_err_num;          //!<堵转�????测允???的�?�忍??????
    uint8_t block_flag = 0;       //!<处于检测堵???的标志，不堵??????动恢复为0
};

class manager //电机�????抽象�?
{
public:
    //!< �????性改???，在线列表的数据类型???改到int16_t
    //!<CAN1在线的ID列表�????性更新，适配GM6020的ID会超???0x208，现在[0:10]分别???0x201???0x20B电机在线�???
    static int16_t CAN1_OnlineID; //!<CAN1在线的ID列表，[0:7]分别???0x201???0x207的电机在线状???(�????�?旧的说明)
    //!<CAN2在线的ID列表�????性更新，适配GM6020的ID会超???0x208，现在[0:10]分别???0x201???0x20B电机在线�???
    static int16_t CAN2_OnlineID; //!<CAN2在线的ID列表，[0:7]分别???0x201???0x207的电机在线状???(�????�?旧的说明)
    RunState_t RunState = Stop;   //!<电机的运行状???
    //!电机???否是协同工作的，如果属于某机构的一部分，则该值可以置1�????时Handle()函数不会由CANSend�????，需要自己实现，从而完成电机组合逻辑
    uint8_t cooperative;
    uint8_t Is_Offline(void); //!<判断当前电机???�????于�?�线�???
    //!全局的CAN收发函数,必须必须必须先调用CANSelect()且必须在CAN接收打开前调用本函数???认接收句??? Update函数放在CAN接收回调里面,Send保证周期�????,
    static void CANSelect(CAN_HandleTypeDef *canhandle1, CAN_HandleTypeDef *canhandle2);
    static void CANUpdate(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data); //!<全局的接�????理函�?,统一管配所有电???
    static void UserProcess(void);                                                               //!<在PID跑完 发送前�????的数�?处理函数，建???以重写的形式�????新的???�?
    static uint8_t CANSend(void);                                                                //!<全局的发送函数，统一管配所有电???
    void Speed_F_Set(float f);                                                                   //!<设定�????�? 输入电机的新�???
    virtual void Safe_Set(void) = 0;                                                             //!<子类必须实现的纯虚函???
protected:
    uint8_t can_code;                     //!<CAN???,以十进制存储,百位数显示是can几，后面两位数表示id???-1，范???0~10
    static CAN_HandleTypeDef *CanHandle1; //!<CAN�????1的指???
    static CAN_HandleTypeDef *CanHandle2; //!<CAN�????2的指???
    static manager *CAN1MotorList[11];    //!<CAN1电机地址列表，由8改动???11
    static int16_t CAN1CurrentList[11];   //!<CAN1电机待发送电流列???，由8改动???11
    static manager *CAN2MotorList[11];    //!<CAN2电机地址列表，由8改动???11
    static int16_t CAN2CurrentList[11];   //!<CAN2电流列表，由8改动???11
    uint32_t LastUpdateTime;              //!<�????更新的时�?
    float Speed_LPF;                      //!<速度�????低通滤波器
    float Speed_F;                        //!<速度�????�????

    virtual void update(uint8_t Data[]) = 0; //!<子类必须实现的纯虚函???
    virtual void Handle(void) = 0;           //!<子类必须实现的发�????理函�?
};
class motor : public manager //!???通电机类???
{
public:
    int16_t RealCurrent;      //编码器反馈转矩电???
    int16_t LastRealCurrent;  //编码器反馈转矩电???
    float RealAngle;          //!<根据�??????????算出的真实�?�度
    int16_t TargetCurrent;    //!<???标电�???
    int16_t RealPosition;     //!<真实位置(编码???)
    int16_t TargetPosition;   //!<???标位???
    int16_t RealSpeed;        //!<实际速度(编码???)
    int16_t TargetSpeed;      //!<???标速度
    Motor_t *MotorType;       //!<电机(电调)参数
    block_type *block = NULL; //!<堵转对象指针，在使用堵转检测时会生�????象并储存指针在这�?
    motor(void){};            //!<仅用于默认构造函???
    motor(uint8_t can_num,
          uint16_t _can_id,
          Motor_t *motor_type,
          pid *_PID_In,
          pid *_PID_Out = NULL);                                                  //!<构造方式之一，只提供速度环pid
    void Speed_Set(int16_t);                                                      //!<设定速度，其实可以直�????定TargetSpeed
    void Angle_Set(float);                                                        //!<设定位置，其实可以直�????定TargetPosition
    virtual int8_t Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num); //!<到时候写??? 注意负数 ???坑了
    virtual void Safe_Set(void);                                                  //!<设定电机进入安全模式，即发送电�???0
protected:
    class pid *PID_In;    //!<PID内环
    class pid *PID_Out;   //!<PID外环
    int16_t LastPosition; //!<�????�????
    int16_t LastSpeed;    //!<�????速度

    virtual void update(uint8_t Data[]); //!<直接Data数组的update函数
    virtual void Handle(void);           //!<数据处理函数，用于判???状态，�????PID
    virtual void Position_Run(void);     //!<使用位置??????定速度 为PID运算??????
    virtual void Speed_Run(void);        //!<使用速度??????定电??? 为PID运算??????
public:
    virtual void InsertCurrent(void);    //!<将运算好的电流按列表储存进发送缓存区???
	virtual void InsertCurrentBy(int16_t tar_cur);
};
/** 
    * @brief ??????程电??? \n
		* �????�????:把电机面对自??? 逆时针转动的�??? ???�????数�?�加 
    */
class softmotor : public motor
{
    friend class chassis; //声明底盘类型为电机的友元类，让底盘能�????�?电机�????有成�?
public:
    int32_t Soft_RealPosition = 0; //!<???真实???程，这里实际意义??????子转过的圈数
    int32_t Soft_TargetPosition;   //!<??????标路程，实际意义为轮�????�?的圈???
    float SoftAngle;               //!<???角度，根??????过圈数来�???????�? RealAngle现在???单圈�????�????

    softmotor() : motor() {} //避免在构造空对象时不成功
    softmotor(uint8_t can_num,
              uint16_t _can_id,
              Motor_t *motor_type,
              pid *PID_In,
              pid *PID_Out = NULL)
        : motor(can_num, _can_id, motor_type, PID_In, PID_Out) {}                 //!<构造函???
    void Limit(float _max, float _min);                                           //!<设置???件限???
    void Angle_Set(float);                                                        //!<设置???程目�????�?
    virtual int8_t Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num); //!<到时候写??? 注意负数 ???坑了
protected:
    virtual void update(uint8_t Data[]);
    virtual void Position_Run(void);
    float max = 99999999999;  //!<角度最�???
    float min = -99999999999; //!<角度最�???
private:
    uint8_t running_flag = 0; //!<用来保证???一次的??????程不突变 因为开机时LastPosition???定是0???能会导致一�????�?
};
class cloud : public manager //!扩展:云台???(6623)
{
public:
    Motor_t *MotorType; //!<电机(电调)参数
    float RealAngle;    //!<根据�??????????算出的真实�?�度
    //电流???
    int16_t TargetCurrent; //!<发送给电机的电�???
    int16_t RealCurrent;   //!<实际???矩电???(编码???)
    //速度???
    float RealSpeed;        //!<实际速度(陀螺仪/编码???)
    float TargetSpeed;      //!<???标速度
    float *Gyro_RealSpeed;  //!<指向陀螺仪�????速度的指???
    float Gyro_TargetSpeed; //!<陀螺仪设置???标转???
    //位置???
    int16_t RealPosition;     //!<真实位置(编码???)
    int16_t OriginalPosition; //!<�????�????(编码???)，没有经过校???
    int16_t TargetPosition;   //!<???标位???
    float *Gyro_RealAngle;    //!<指向陀螺仪�???????度的�????
    float Gyro_TargetAngle;   //!<陀螺仪设置???�????�?

    pid *PID_In;       //!<�???????PID速度???
    pid *PID_Out;      //!<�???????PID位置???
    pid *Gyro_PID_In;  //!<陀螺仪PID速度???
    pid *Gyro_PID_Out; //!<陀螺仪PID位置???

    void Pid_Select(pid *PID_In_Select, pid *PID_Out_Select);                //!<�???????pid选择
    void Gyro_Pid_Select(pid *Gyro_PID_In_Select, pid *Gyro_PID_Out_Select); //!<陀螺仪pid选择
    void Speed_Set(float);                                                   //!<设定速度，通过�???????�????
    void Angle_Set(float);                                                   //!<设定角度，通过�???????�????
    void Gyro_Speed_Set(float TargetSpeed);                                  //!<设定速度，通过陀螺仪调节
    void Gyro_Angle_Set(float TargetPosition);                               //!<设定角度，通过陀螺仪调节
    void Limit(float _max, float _min);                                      //!<设置???件限???
    void Gyro_Speed_Run(void);                                               //!PID运算函数，manager类调???
    void Gyro_Position_Run(void);                                            //!PID运算函数，manager类调???
    cloud();                                                                 //!<�????构造函???
    cloud(uint8_t can_num,
          uint16_t _can_id,
          int16_t _CLOUD_STD,
          Motor_t *motor_type,
          pid *PID_I,
          pid *PID_O,
          pid *G_In,
          pid *G_Out,
          float *SpeedSource = NULL,
          float *PositionSource = NULL); //!<构造函数，指针指向位置或速度???
    virtual void Safe_Set(void);         //!<设定电机进入安全模式，即发送电�???0

protected:
    int16_t CLOUD_STD; //!<该云台在指向原点时的编码器的???

    int16_t LastTorque;                  //!<�????�????
    float LastSpeed;                     //!<�????速度(�???????)
    int16_t LastPosition;                //!<�????�????(�???????)
    float Gyro_LastSpeed;                //!<�????速度(陀螺仪)
    float Gyro_LastPosition;             //!<�????�????(陀螺仪)
    float max = 99999999999;             //!<角度最�???
    float min = -99999999999;            //!<角度最�???
    virtual void update(uint8_t Data[]); //!<直接Data数组的update函数
    virtual void Handle(void);           //!<数据处理函数，用于判???状态，�????PID
    virtual void Speed_Run(void);        //!<使用速度??????定电??? 为PID运算??????
    virtual void Position_Run(void);     //!<使用位置??????定电??? 为PID运算??????
    virtual void InsertCurrent(void); //!<将运算好的电流按列表储存进发送缓存区???
};
class softcloud : public cloud ///??????程云台类 for 6020
{
public:
    float TargetAngle;             //!<根据�??????????算出的真实�?�度
    int32_t Soft_RealPosition = 0; //!<???真实???程，这里实际意义???云台???过的圈数
    int32_t Soft_TargetPosition;   //!<??????标路程，实际意义为轮�????�?的圈???

    softcloud() : cloud() {} //避免在构造空对象时不成功
    softcloud(uint8_t can_num,
              uint16_t _can_id,
              int16_t _CLOUD_STD,
              Motor_t *motor_type,
              pid *PID_I,
              pid *PID_O,
              pid *G_In,
              pid *G_Out,
              float *SpeedSource = NULL,
              float *PositionSource = NULL)
        : cloud(can_num, _can_id, _CLOUD_STD, motor_type, PID_I, PID_O, G_In, G_Out, SpeedSource, PositionSource), SOFTCLOUD_STD(_CLOUD_STD) {} //!<构造函数，指针指向位置或速度???
    void Angle_Set(float);                                                                                                                      //!<设置???程目�????�?
private:
    virtual void Handle(void);           //!<数据处理函数，用于判???状态，�????PID
    virtual void update(uint8_t Data[]); //!<直接Data数组的update函数
    virtual void Position_Run(void);
    int16_t SOFTCLOUD_STD;    //!<该云台在指向原点时的编码器的???
    uint8_t running_flag = 0; //!<用来保证???一次的??????程不突变 因为开机时LastPosition???定是0???能会导致一�????�?
    //		float max=99999999999;//!<角度最�???
    //		float min=-99999999999;//!<角度最�???
};
/** 
* @brief  底盘的控制类型，带功率控???
* @par 日志 
*   2019???3???9???16:43:30 WMD 因为需要在速度???和电流环之间插入功率限制调整函数，经过考虑重新???改了一�????架，将电�????嵌入底盘类型，而普通电机类型不涉及电流???
*/
class chassis
{
public:
    float extra_power = 0; //利用缓冲能量计算出的�????�????
    pid *Pid_extra_power;  //利用缓冲能量计算�????功率的pid的指???

    static chassis *point; //!<指向当前已声明的底盘，一???工程???�????一???底盘的存???  该指针用于日后托�????�?
    //pid *Turn_PID;  //!<???弯PID
    void Run(void);                            //!<缺省参数 以上次的模式控制底盘
    void Run(float Vx, float Vy, float Omega); //!<以速度控制底盘
    void Safe(void);                           //!�????�????
    softmotor *Motor[4];                       //!<四个???子的电机对象
    chassis(uint8_t can_num,
            uint16_t First_can_id,
            Motor_t *motor_type,
            pid *speed_pid,
            //pid* turnpid=NULL,
            pid *current_pid = NULL,
            int16_t *CurrentSource = NULL,
            pid *extra_power_pid = NULL); //!<直接控制底盘的构造函???
    void Handle(void);                    //!<交由CANSend�????的底盘�?�理函数
private:
    RunState_t RunState;                            //!<当前底盘�???
    pid *Pid_spe[4];                                //!<指向四个电机速度环pid的指???
    pid *Pid_current[4];                            //!<指向四个电机电流环pid的指???
    int16_t *CurrentSource[4];                      //!<指向电流真实数据???
    float Last_Vx = 0, Last_Vy = 0, Last_Omega = 0; //!<之前的值，用于缺省参数时的使用
};
class chassiscontrol //底盘控制类for云台
{
public:
    CAN_HandleTypeDef *Canhandle;
    uint16_t Chassis_ID;
    chassiscontrol(CAN_HandleTypeDef *canhandle, uint16_t chassis_id); //构造函???
    void Run(void);                                                    //!<缺省参数 以上次的模式控制底盘
    void Run(int16_t Vx, int16_t Vy, int16_t Omega, uint8_t Mode,
             uint8_t All_flags = 0);
    void Safe(void); //!底盘安全模式
private:
    int16_t Last_Vx = 0, Last_Vy = 0, Last_Omega = 0; //之前的值，用于缺省参数时的使用
    uint8_t Last_Mode = 22;
};



#endif
