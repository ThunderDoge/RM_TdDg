/** 
  * @file         bsp_motor.hpp 
  * @author      WMD,Onion rain
  *             Evan-GH
  *             ThunderDoge
  * @version  7.7b
  * @par Copyright (c):  
  *       WMD,Onion rain
  *				Evan-GH
  * @par ��־��cpp
  */
#ifndef __BSP_MOTOR_HPP
#define __BSP_MOTOR_HPP

#include "sentry_config.hpp"

#include "bsp_car_config.hpp"
using namespace std;

typedef struct str_Motor_t
{
    str_Motor_t(){}; //���ⴴ��?????��?????
    str_Motor_t(uint16_t a, uint8_t b) : max_mechanical_position(a), Reduction_ratio(b){};
    uint16_t max_mechanical_position = 8192; //!<????????��????,?????0x2000
    uint8_t Reduction_ratio = 19;            //!<���ٱ�,?????19:1
} Motor_t;                                   //!<���(���)�����ṹ�壬?????ֵΪ3508�������?????C620���

class pid; //������һ?? by thunderdoge 2020-2-27
/**
 * @brief ��������PID���û��Զ���ص�������ָ??
 */
typedef void (*pvPidCallBack)(pid*);
//PID???����pid��� ???������pid
class pid
{
public:
    float *Custom_Diff = NULL; //!<???����??????????����???����??? ������·�̻���΢�ֻ�???(���ٶ�???)
    // pvPidCallBack pid_run_CallBack=nullptr; 
    uint16_t I_Time;           //!<pidʱ����� ???msΪ��??? plusר��,???��ʱ???
    uint16_t D_Time;           //!<???��ʱ�䣬msΪ��???
    uint16_t I_Limited;        //!<????????С��I_Limitedʱ��?????I��� plusר��
    float ap = 0, bp = 0, cp;  //ap==0 && bp==0˵�����Ƿ���???pid
    float ai = 0, ci;
    float ad = 0, bd = 0, cd, dd;
    float P;
    float I;
    float D;
    float IMax;
    float PIDMax;

    //???����???
    pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time = 1, uint16_t D_Time = 1, uint16_t I_Limited = 9999); //��ͳpid���캯???
    pid(float ap, float bp, float cp,
        float ai, float ci,
        float ad, float bd, float cd, float dd,
        float IMax, float PIDMax, uint16_t I_Time = 1, uint16_t D_Time = 1, uint16_t I_Limited = 9999); //����???pid���캯???
    float pid_run(float err);
    float nonlinear_pid_run(float err);
    float sech(float in);

//private:
    //���㴢��???
    float Pout;
    float Iout;
    float Dout;
    float Dout_Accumulative; //��Ϊ��΢��ʱ���ض���,?????����һ???����?????�������D,ԴDout����????????????
    float PIDout;
    float CurrentError;
    float LastError;
    uint32_t I_start_time; //!<???�ֿ�ʼʱ��������ڴ�ʱ�������pid   plusר��
    uint32_t D_start_time; //!<???�ֿ�ʼʱ��������ڴ�ʱ�������pid

    static void PIDUserProcess(void);  //!< ��pid_run����ǰ���е��û����ݴ�����
};

/** 
	* @brief ��ת����� 
	* @details Ҫ���ڳ�ʼ����ʱ�������?????���??�ĵ������ú�λ������ \n
	*		  һ???���ᵥ��ʹ�ø��࣬��??????????�ֶ�???��⹦�ܵĻ��������ڲ���һ???��ת����ָ�� \n
	*		  ???????�õ���Ķ�ת��⹦�ܵĻ�ʹ??????Ӧ��������ṩ�ĳ�???������ \n
	*		  �����⻹��Ҫ�ṩ��???�϶���������???�϶�ʱ�䣬��???ʱ��������ĵ���Ƕ�ƫ��??? \n
	*		  ???��ͨ�� @see block_type::IsBlock ������ȷ�ϵ���Ƿ��??? \n 
	*		  ???��ͨ�� @see block_type::Clear_BlockFlag() �������???��־
*/
class block_type
{
public:
    void Block_Init(uint16_t Limit, uint16_t time, float err_num);                                 //!<??????????����???????
    block_type(int16_t &_Current, float &_RealAngle) : Current(_Current), RealAngle(_RealAngle) {} //!���캯???
    void Block_Check(void);                                                                        //!<����???�ĺ�???
    void Clear_BlockFlag(void);                                                                    //!<ȥ����ת��־
    uint8_t IsBlock;                                                                               //!<???���???�ı�־��1Ϊ�Ѿ���???����Ҫ��Ϊʹ��Clear_BlockFlag����???��־
private:
    int16_t &Current;
    float &RealAngle;
    float block_Angle;            //!<��תʱ��λ��
    uint16_t block_Current_Limit; //!<��ת���������޵�???
    uint32_t block_time;          //!<��ת��ʼʱ��ʱ���
    uint32_t block_Time_Limit;    //!<��ת���ʱ?????�Ķ�??ʱ��
    float block_err_num;          //!<��ת?????����???��???��??????
    uint8_t block_flag = 0;       //!<���ڼ���???�ı�־������??????���ָ�Ϊ0
};

class manager //���?????����??
{
public:
    //!< ?????�Ը�???�������б����������???�ĵ�int16_t
    //!<CAN1���ߵ�ID�б�?????�Ը��£�����GM6020��ID�ᳬ???0x208������[0:10]�ֱ�???0x201???0x20B�������????
    static int16_t CAN1_OnlineID; //!<CAN1���ߵ�ID�б�[0:7]�ֱ�???0x201???0x207�ĵ������״???(???????�ɵ�˵��)
    //!<CAN2���ߵ�ID�б�?????�Ը��£�����GM6020��ID�ᳬ???0x208������[0:10]�ֱ�???0x201???0x20B�������????
    static int16_t CAN2_OnlineID; //!<CAN2���ߵ�ID�б�[0:7]�ֱ�???0x201???0x207�ĵ������״???(???????�ɵ�˵��)
    RunState_t RunState = Stop;   //!<���������״???
    //!���???����Эͬ�����ģ��������ĳ������һ���֣����ֵ������1?????ʱHandle()����������CANSend?????����Ҫ�Լ�ʵ�֣��Ӷ���ɵ������߼�
    uint8_t cooperative;
    uint8_t Is_Offline(void); //!<�жϵ�ǰ���????????��???��????
    //!ȫ�ֵ�CAN�շ�����,�����������ȵ���CANSelect()�ұ�����CAN���մ�ǰ���ñ�����???�Ͻ��վ�??? Update��������CAN���ջص�����,Send��֤����?????,
    static void CANSelect(CAN_HandleTypeDef *canhandle1, CAN_HandleTypeDef *canhandle2);
    static void CANUpdate(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data); //!<ȫ�ֵĽ�?????��??,ͳһ�������е�???
    static void UserProcess(void);                                                               //!<��PID���� ����ǰ?????����??����������???����д����ʽ?????�µ�?????
    static uint8_t CANSend(void);                                                                //!<ȫ�ֵķ��ͺ�����ͳһ�������е�???
    void Speed_F_Set(float f);                                                                   //!<�趨??????? ����������????
    virtual void Safe_Set(void) = 0;                                                             //!<�������ʵ�ֵĴ��麯???
    uint32_t LastUpdateTime;              //!<?????���µ�ʱ??
protected:
    uint8_t can_code;                     //!<CAN???,��ʮ���ƴ洢,��λ����ʾ��can����������λ����ʾid???-1����???0~10
    static CAN_HandleTypeDef *CanHandle1; //!<CAN?????1��ָ???
    static CAN_HandleTypeDef *CanHandle2; //!<CAN?????2��ָ???
    static manager *CAN1MotorList[11];    //!<CAN1�����ַ�б���8�Ķ�???11
    static int16_t CAN1CurrentList[11];   //!<CAN1��������͵�����???����8�Ķ�???11
    static manager *CAN2MotorList[11];    //!<CAN2�����ַ�б���8�Ķ�???11
    static int16_t CAN2CurrentList[11];   //!<CAN2�����б���8�Ķ�???11
    float Speed_LPF;                      //!<�ٶ�?????��ͨ�˲���
    float Speed_F;                        //!<�ٶ�??????????

    virtual void update(uint8_t Data[]) = 0; //!<�������ʵ�ֵĴ��麯???
    virtual void Handle(void) = 0;           //!<�������ʵ�ֵķ�?????��??
};
class motor : public manager //!???ͨ�����???
{
public:
    int16_t RealCurrent;      //����������ת�ص�???
    int16_t LastRealCurrent;  //����������ת�ص�???
    float RealAngle;          //!<����???????????�������ʵ???��
    int16_t TargetCurrent;    //!<???���????
    int16_t RealPosition;     //!<��ʵλ��(����???)
    int16_t TargetPosition;   //!<???��λ???
    int16_t RealSpeed;        //!<ʵ���ٶ�(����???)
    int16_t TargetSpeed;      //!<???���ٶ�
    Motor_t *MotorType;       //!<���(���)����
    block_type *block = NULL; //!<��ת����ָ�룬��ʹ�ö�ת���ʱ����?????�󲢴���ָ������??
    motor(void){};            //!<������Ĭ�Ϲ��캯???
    motor(uint8_t can_num,
          uint16_t _can_id,
          Motor_t *motor_type,
          pid *_PID_In,
          pid *_PID_Out = NULL);                                                  //!<���췽ʽ֮һ��ֻ�ṩ�ٶȻ�pid
    void Speed_Set(int16_t);                                                      //!<�趨�ٶȣ���ʵ����ֱ?????��TargetSpeed
    void Angle_Set(float);                                                        //!<�趨λ�ã���ʵ����ֱ?????��TargetPosition
    virtual int8_t Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num); //!<��ʱ��д??? ע�⸺�� ???����
    virtual void Safe_Set(void);                                                  //!<�趨������밲ȫģʽ�������͵�????0
protected:
    class pid *PID_In;    //!<PID�ڻ�
    class pid *PID_Out;   //!<PID�⻷
    int16_t LastPosition; //!<??????????
    int16_t LastSpeed;    //!<?????�ٶ�

    virtual void update(uint8_t Data[]); //!<ֱ��Data�����update����
    virtual void Handle(void);           //!<���ݴ�������������???״̬��?????PID
    virtual void Position_Run(void);     //!<ʹ��λ��??????���ٶ� ΪPID����??????
    virtual void Speed_Run(void);        //!<ʹ���ٶ�??????����??? ΪPID����??????
public:
    virtual void InsertCurrent(void);    //!<������õĵ������б�������ͻ�����???
	virtual void InsertCurrentBy(int16_t tar_cur);
};
/** 
    * @brief ??????�̵�??? \n
		* ??????????:�ѵ�������??? ��ʱ��ת����???? ????????��???�� 
    */
class softmotor : public motor
{
    friend class chassis; //������������Ϊ�������Ԫ�࣬�õ�����???????���?????�г�??
public:
    int32_t Soft_RealPosition = 0; //!<???��ʵ???�̣�����ʵ������??????��ת����Ȧ��
    int32_t Soft_TargetPosition;   //!<??????��·�̣�ʵ������Ϊ��???????��Ȧ???
    float SoftAngle;               //!<???�Ƕȣ���??????��Ȧ����?????????? RealAngle����???��Ȧ??????????

    softmotor() : motor() {} //�����ڹ���ն���ʱ���ɹ�
    softmotor(uint8_t can_num,
              uint16_t _can_id,
              Motor_t *motor_type,
              pid *PID_In,
              pid *PID_Out = NULL)
        : motor(can_num, _can_id, motor_type, PID_In, PID_Out) {}                 //!<���캯???
    void Limit(float _max, float _min);                                           //!<����???����???
    void Angle_Set(float);                                                        //!<����???��Ŀ???????
    virtual int8_t Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num); //!<��ʱ��д??? ע�⸺�� ???����
	void ForceSetSoftAngle(float);
protected:
    virtual void update(uint8_t Data[]);
    virtual void Position_Run(void);
    float max = 99999999999;  //!<�Ƕ���????
    float min = -99999999999; //!<�Ƕ���????
private:
    uint8_t running_flag = 0; //!<������֤???һ�ε�??????�̲�ͻ�� ��Ϊ����ʱLastPosition???����0???�ܻᵼ��һ???????
};
class cloud : public manager //!��չ:��̨???(6623)
{
public:
    Motor_t *MotorType; //!<���(���)����
    float RealAngle;    //!<����???????????�������ʵ???��
    //����???
    int16_t TargetCurrent; //!<���͸�����ĵ�????
    int16_t RealCurrent;   //!<ʵ��???�ص�???(����???)
    //�ٶ�???
    float RealSpeed;        //!<ʵ���ٶ�(������/����???)
    float TargetSpeed;      //!<???���ٶ�
    float *Gyro_RealSpeed;  //!<ָ��������?????�ٶȵ�ָ???
    float Gyro_TargetSpeed; //!<����������???��ת???
    //λ��???
    int16_t RealPosition;     //!<��ʵλ��(����???)
    int16_t OriginalPosition; //!<??????????(����???)��û�о���У???
    int16_t TargetPosition;   //!<???��λ???
    float *Gyro_RealAngle;    //!<ָ��������????????�ȵ�?????
    float Gyro_TargetAngle;   //!<����������??????????

    pid *PID_In;       //!<????????PID�ٶ�???
    pid *PID_Out;      //!<????????PIDλ��???
    pid *Gyro_PID_In;  //!<������PID�ٶ�???
    pid *Gyro_PID_Out; //!<������PIDλ��???

    void Pid_Select(pid *PID_In_Select, pid *PID_Out_Select);                //!<????????pidѡ��
    void Gyro_Pid_Select(pid *Gyro_PID_In_Select, pid *Gyro_PID_Out_Select); //!<������pidѡ��
    void Speed_Set(float);                                                   //!<�趨�ٶȣ�ͨ��?????????????
    void Angle_Set(float);                                                   //!<�趨�Ƕȣ�ͨ��?????????????
    void Gyro_Speed_Set(float TargetSpeed);                                  //!<�趨�ٶȣ�ͨ�������ǵ���
    void Gyro_Angle_Set(float TargetPosition);                               //!<�趨�Ƕȣ�ͨ�������ǵ���
    void Limit(float _max, float _min);                                      //!<����???����???
    void Gyro_Speed_Run(void);                                               //!PID���㺯����manager���???
    void Gyro_Position_Run(void);                                            //!PID���㺯����manager���???
    cloud();                                                                 //!<?????���캯???
    cloud(uint8_t can_num,
          uint16_t _can_id,
          int16_t _CLOUD_STD,
          Motor_t *motor_type,
          pid *PID_I,
          pid *PID_O,
          pid *G_In,
          pid *G_Out,
          float *SpeedSource = NULL,
          float *PositionSource = NULL); //!<���캯����ָ��ָ��λ�û��ٶ�???
    virtual void Safe_Set(void);         //!<�趨������밲ȫģʽ�������͵�????0
    virtual void InsertCurrent(void); //!<������õĵ������б�������ͻ�����???
protected:
    int16_t CLOUD_STD; //!<����̨��ָ��ԭ��ʱ�ı�������???

    int16_t LastTorque;                  //!<??????????
    float LastSpeed;                     //!<?????�ٶ�(????????)
    int16_t LastPosition;                //!<??????????(????????)
    float Gyro_LastSpeed;                //!<?????�ٶ�(������)
    float Gyro_LastPosition;             //!<??????????(������)
    float max = 99999999999;             //!<�Ƕ���????
    float min = -99999999999;            //!<�Ƕ���????
    virtual void update(uint8_t Data[]); //!<ֱ��Data�����update����
    virtual void Handle(void);           //!<���ݴ�������������???״̬��?????PID
    virtual void Speed_Run(void);        //!<ʹ���ٶ�??????����??? ΪPID����??????
    virtual void Position_Run(void);     //!<ʹ��λ��??????����??? ΪPID����??????
};
class softcloud : public cloud ///??????����̨�� for 6020
{
public:
    float TargetAngle;             //!<����???????????�������ʵ???��
    int32_t Soft_RealPosition = 0; //!<???��ʵ???�̣�����ʵ������???��̨???����Ȧ��
    int32_t Soft_TargetPosition;   //!<??????��·�̣�ʵ������Ϊ��???????��Ȧ???

    softcloud() : cloud() {} //�����ڹ���ն���ʱ���ɹ�
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
        : cloud(can_num, _can_id, _CLOUD_STD, motor_type, PID_I, PID_O, G_In, G_Out, SpeedSource, PositionSource), SOFTCLOUD_STD(_CLOUD_STD) {} //!<���캯����ָ��ָ��λ�û��ٶ�???
    void Angle_Set(float);                                                                                                                      //!<����???��Ŀ???????
private:
    virtual void Handle(void);           //!<���ݴ�������������???״̬��?????PID
    virtual void update(uint8_t Data[]); //!<ֱ��Data�����update����
    virtual void Position_Run(void);
    int16_t SOFTCLOUD_STD;    //!<����̨��ָ��ԭ��ʱ�ı�������???
    uint8_t running_flag = 0; //!<������֤???һ�ε�??????�̲�ͻ�� ��Ϊ����ʱLastPosition???����0???�ܻᵼ��һ???????
    //		float max=99999999999;//!<�Ƕ���????
    //		float min=-99999999999;//!<�Ƕ���????
};
/** 
* @brief  ���̵Ŀ������ͣ������ʿ�???
* @par ��־ 
*   2019???3???9???16:43:30 WMD ��Ϊ��Ҫ���ٶ�???�͵�����֮����빦�����Ƶ���������������������???����һ?????�ܣ�����?????Ƕ��������ͣ�����ͨ������Ͳ��漰����???
*/
class chassis
{
public:
    float extra_power = 0; //���û��������������??????????
    pid *Pid_extra_power;  //���û�����������?????���ʵ�pid��ָ???

    static chassis *point; //!<ָ��ǰ�������ĵ��̣�һ???����????????һ???���̵Ĵ�???  ��ָ�������պ���???????
    //pid *Turn_PID;  //!<???��PID
    void Run(void);                            //!<ȱʡ���� ���ϴε�ģʽ���Ƶ���
    void Run(float Vx, float Vy, float Omega); //!<���ٶȿ��Ƶ���
    void Safe(void);                           //!??????????
    softmotor *Motor[4];                       //!<�ĸ�???�ӵĵ������
    chassis(uint8_t can_num,
            uint16_t First_can_id,
            Motor_t *motor_type,
            pid *speed_pid,
            //pid* turnpid=NULL,
            pid *current_pid = NULL,
            int16_t *CurrentSource = NULL,
            pid *extra_power_pid = NULL); //!<ֱ�ӿ��Ƶ��̵Ĺ��캯???
    void Handle(void);                    //!<����CANSend?????�ĵ���???����
private:
    RunState_t RunState;                            //!<��ǰ����????
    pid *Pid_spe[4];                                //!<ָ���ĸ�����ٶȻ�pid��ָ???
    pid *Pid_current[4];                            //!<ָ���ĸ����������pid��ָ???
    int16_t *CurrentSource[4];                      //!<ָ�������ʵ����???
    float Last_Vx = 0, Last_Vy = 0, Last_Omega = 0; //!<֮ǰ��ֵ������ȱʡ����ʱ��ʹ��
};
class chassiscontrol //���̿�����for��̨
{
public:
    CAN_HandleTypeDef *Canhandle;
    uint16_t Chassis_ID;
    chassiscontrol(CAN_HandleTypeDef *canhandle, uint16_t chassis_id); //���캯???
    void Run(void);                                                    //!<ȱʡ���� ���ϴε�ģʽ���Ƶ���
    void Run(int16_t Vx, int16_t Vy, int16_t Omega, uint8_t Mode,
             uint8_t All_flags = 0);
    void Safe(void); //!���̰�ȫģʽ
private:
    int16_t Last_Vx = 0, Last_Vy = 0, Last_Omega = 0; //֮ǰ��ֵ������ȱʡ����ʱ��ʹ��
    uint8_t Last_Mode = 22;
};



#endif
