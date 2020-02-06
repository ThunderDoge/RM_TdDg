/** 
* @file         bsp_motor.hpp 
* @author      WMD,Onion rain
*								Evan-GH
* @version  7.5b
* @par Copyright (c):  
*       WMD,Onion rain
*				Evan-GH
* @par ��־��cpp
*/ 
#ifndef __BSP_MOTOR_HPP
#define __BSP_MOTOR_HPP
#include "bsp_car_config.hpp"
using namespace std;

typedef struct str_Motor_t
{
	str_Motor_t(){};//���ⴴ���ն��󲻳ɹ�
	str_Motor_t(uint16_t a,uint8_t b):max_mechanical_position(a),Reduction_ratio(b){};
	uint16_t max_mechanical_position = 8192;//!<��е�����ֵ,Ĭ��0x2000
	uint8_t Reduction_ratio = 19;        //!<���ٱ�,Ĭ��19:1
}Motor_t;//!<���(���)�����ṹ�壬Ĭ��ֵΪ3508�����������C620���

//PID������pid��� ��������pid
class pid
{
	public:
		float* Custom_Diff=NULL;//!<�Զ������縡����΢����ֵ ������·�̻���΢�ֻ���(���ٶ�ֵ)
		uint16_t I_Time;//!<pidʱ����� ��msΪ��λ plusר��,����ʱ��
		uint16_t D_Time;//!<΢��ʱ�䣬msΪ��λ
		uint16_t I_Limited; //!<�����С��I_Limitedʱ�Ž���I��� plusר��
		float ap = 0, bp = 0, cp;//ap==0 && bp==0˵�����Ƿ�����pid
		float ai = 0, ci;
		float ad = 0, bd = 0, cd, dd;
		float P;
		float I;
		float D;
		float IMax;
		float PIDMax;

		//��������
		pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//��ͳpid���캯��
		pid(float ap, float bp, float cp,
				float ai,           float ci,
				float ad, float bd, float cd, float dd,
				float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1, uint16_t I_Limited=9999);//������pid���캯��
		float pid_run(float err);
		float nonlinear_pid_run(float err);
		float sech(float in);
	private:
		//���㴢����
		float Pout;
		float Iout;
		float Dout;
		float Dout_Accumulative;//��Ϊ��΢��ʱ���ض���,��˼���һ�����������������D,ԴDout�����ۻ����
		float PIDout;
		float CurrentError;
		float LastError;
		uint32_t I_start_time;//!<���ֿ�ʼʱ��������ڴ�ʱ�������pid   plusר��
		uint32_t D_start_time;//!<΢�ֿ�ʼʱ��������ڴ�ʱ�������pid 
};

/** 
	* @brief ��ת����� 
	* @details Ҫ���ڳ�ʼ����ʱ�������Ҫ����ת�ĵ������ú�λ������ \n
	*		  һ�㲻�ᵥ��ʹ�ø��࣬������֧�ֶ�ת��⹦�ܵĻ��������ڲ���һ����ת����ָ�� \n
	*		  ��Ҫ���õ���Ķ�ת��⹦�ܵĻ�ʹ�ö�Ӧ��������ṩ�ĳ�ʼ������ \n
	*		  �����⻹��Ҫ�ṩ��ת�϶���������ת�϶�ʱ�䣬��תʱ��������ĵ���Ƕ�ƫ���� \n
	*		  ����ͨ�� @see block_type::IsBlock ������ȷ�ϵ���Ƿ��ת \n 
	*		  ����ͨ�� @see block_type::Clear_BlockFlag() �������ת��־
*/
class block_type
{
	public:
		void Block_Init(uint16_t Limit, uint16_t time, float err_num);//!<���ж�ת������ʼ��
		block_type(int16_t& _Current,float& _RealAngle):Current(_Current),RealAngle(_RealAngle){}//!���캯��
		void Block_Check(void);     //!<����ת�ĺ���
		void Clear_BlockFlag(void); //!<ȥ����ת��־
		uint8_t IsBlock;            //!<�Ƿ��ת�ı�־��1Ϊ�Ѿ���ת����Ҫ��Ϊʹ��Clear_BlockFlag����ת��־
	private:
		int16_t& Current;
		float& RealAngle;
		float block_Angle;            //!<��תʱ��λ��
		uint16_t block_Current_Limit; //!<��ת���������޵���
		uint32_t block_time;          //!<��ת��ʼʱ��ʱ���
		uint32_t block_Time_Limit;    //!<��ת���ʱ����Ķ�תʱ��
		float    block_err_num;       //!<��תʱ���������������
		uint8_t  block_flag=0;        //!<���ڼ���ת�ı�־������ת�Զ��ָ�Ϊ0
};

class manager//����ܼҳ�����
{
	public:
		//!< �����ԸĶ��������б�����������޸ĵ�int16_t
		//!<CAN1���ߵ�ID�б�����Ը��£�����GM6020��ID�ᳬ��0x208������[0:10]�ֱ�Ϊ0x201��0x20B�������״̬
		static int16_t CAN1_OnlineID;//!<CAN1���ߵ�ID�б�[0:7]�ֱ�Ϊ0x201��0x207�ĵ������״̬(�˴��Ǿɵ�˵��)
		//!<CAN2���ߵ�ID�б�����Ը��£�����GM6020��ID�ᳬ��0x208������[0:10]�ֱ�Ϊ0x201��0x20B�������״̬
		static int16_t CAN2_OnlineID;//!<CAN2���ߵ�ID�б�[0:7]�ֱ�Ϊ0x201��0x207�ĵ������״̬(�˴��Ǿɵ�˵��)
		RunState_t RunState=Stop;   //!<���������״̬
		//!����Ƿ���Эͬ�����ģ��������ĳ������һ���֣����ֵ������1����ʱHandle()����������CANSendִ�У���Ҫ�Լ�ʵ�֣��Ӷ���ɵ������߼�
		uint8_t cooperative;          
		uint8_t Is_Offline(void);//!<�жϵ�ǰ����Ƿ�������״̬
		//!ȫ�ֵ�CAN�շ�����,�����������ȵ���CANSelect()�ұ�����CAN���մ�ǰ���ñ�����ȷ�Ͻ��վ�� Update��������CAN���ջص�����,Send��֤����ִ��,
		static void CANSelect(CAN_HandleTypeDef* canhandle1,CAN_HandleTypeDef* canhandle2);
		static void CANUpdate(CAN_HandleTypeDef* _hcan,CAN_RxHeaderTypeDef* RxHead,uint8_t* Data);//!<ȫ�ֵĽ��մ�����,ͳһ�������е��
		static void UserProcess(void);          //!<��PID���� ����ǰ���е����ݴ���������������д����ʽ�����µĴ���
		static uint8_t CANSend(void);          //!<ȫ�ֵķ��ͺ�����ͳһ�������е��
		void Speed_F_Set(float f);             //!<�趨ǰ���� ���������µ�ֵ
		virtual void Safe_Set(void) = 0;       //!<�������ʵ�ֵĴ��麯��
	protected:
		uint8_t can_code;                           //!<CAN��,��ʮ���ƴ洢,��λ����ʾ��can����������λ����ʾid��-1����Χ0~10
		static CAN_HandleTypeDef* CanHandle1;       //!<CAN�豸1��ָ��
		static CAN_HandleTypeDef* CanHandle2;       //!<CAN�豸2��ָ��
		static manager* CAN1MotorList[11];           //!<CAN1�����ַ�б���8�Ķ���11
		static int16_t CAN1CurrentList[11];          //!<CAN1��������͵����б���8�Ķ���11
		static manager* CAN2MotorList[11];           //!<CAN2�����ַ�б���8�Ķ���11
		static int16_t CAN2CurrentList[11];          //!<CAN2�����б���8�Ķ���11
		uint32_t LastUpdateTime;                    //!<�ϴθ��µ�ʱ��
		float Speed_LPF;                            //!<�ٶ�ǰ����ͨ�˲���
		float Speed_F;                              //!<�ٶ�ǰ������
		
		virtual void update(uint8_t Data[]) = 0;    //!<�������ʵ�ֵĴ��麯��
		virtual void Handle(void) = 0;              //!<�������ʵ�ֵķ��͹�����
};
class motor:public manager//!��ͨ�������
{
	public:
		int16_t RealCurrent;			//����������ת�ص���
		int16_t LastRealCurrent;	//����������ת�ص���
		float RealAngle;          //!<���ݻ�е�Ǽ��������ʵ�Ƕ�
		int16_t TargetCurrent;    //!<Ŀ�����ֵ
		int16_t RealPosition;	    //!<��ʵλ��(������)
		int16_t TargetPosition;	  //!<Ŀ��λ��
		int16_t RealSpeed;		    //!<ʵ���ٶ�(������)
		int16_t TargetSpeed;	    //!<Ŀ���ٶ�
		Motor_t *MotorType;       //!<���(���)����
		block_type *block=NULL;   //!<��ת����ָ�룬��ʹ�ö�ת���ʱ�����ɶ��󲢴���ָ��������
		motor(void){};            //!<������Ĭ�Ϲ��캯��
		motor(uint8_t can_num,
					uint16_t _can_id,
					Motor_t *motor_type,
					pid* _PID_In,
					pid* _PID_Out=NULL);//!<���췽ʽ֮һ��ֻ�ṩ�ٶȻ�pid
		void Speed_Set(int16_t);    //!<�趨�ٶȣ���ʵ����ֱ���趨TargetSpeed
		void Angle_Set(float);   //!<�趨λ�ã���ʵ����ֱ���趨TargetPosition
		virtual int8_t Enable_Block(uint16_t Limit,uint16_t time,uint16_t err_num);//!<��ʱ��д�� ע�⸺�� ������
		virtual void Safe_Set(void);//!<�趨������밲ȫģʽ�������͵���ֵ0
	protected:
		class pid* PID_In;          //!<PID�ڻ�
		class pid* PID_Out;         //!<PID�⻷
		int16_t LastPosition;       //!<�ϴ�λ��
		int16_t LastSpeed;          //!<�ϴ��ٶ�

		virtual void update(uint8_t Data[]); //!<ֱ��Data�����update����
		virtual void Handle(void);           //!<���ݴ������������ж�״̬������PID
		virtual void Position_Run(void);     //!<ʹ��λ�û�ȷ���ٶ� ΪPID���㻷��
		virtual void Speed_Run(void);        //!<ʹ���ٶȻ�ȷ������ ΪPID���㻷��
		virtual void InsertCurrent(void);    //!<������õĵ������б�������ͻ�������
};
/** 
    * @brief ��·�̵�� \n
		* ��Ҫ����:�ѵ������Լ� ��ʱ��ת����ʱ�� ·�̼������� 
    */
class	softmotor:public motor
{
	friend class chassis;//������������Ϊ�������Ԫ�࣬�õ����ܹ����ʵ����˽�г�Ա
	public:
		int32_t Soft_RealPosition=0;//!<����ʵ·�̣�����ʵ������������ת����Ȧ��
		int32_t Soft_TargetPosition;//!<��Ŀ��·�̣�ʵ������Ϊ����Ҫת��Ȧ��
		float SoftAngle; //!<��Ƕȣ�����ת��Ȧ�������м��� RealAngle�����ǵ�Ȧ�ڽǶ���

		softmotor():motor(){}//�����ڹ���ն���ʱ���ɹ�
		softmotor(uint8_t can_num,
							uint16_t _can_id,
							Motor_t *motor_type,
							pid* PID_In,
							pid* PID_Out=NULL)
							:motor(can_num, _can_id, motor_type, PID_In, PID_Out){}//!<���캯��
		void Limit(float _max,float _min);//!<���������λ
		void Angle_Set(float);//!<����·��Ŀ��Ƕ�
		virtual int8_t Enable_Block(uint16_t Limit,uint16_t time,uint16_t err_num);//!<��ʱ��д�� ע�⸺�� ������
	protected:
		virtual void update(uint8_t Data[]);
		virtual void Position_Run(void);
		float max=99999999999;//!<�Ƕ����ֵ
		float min=-99999999999;//!<�Ƕ���Сֵ
	private:
		uint8_t running_flag=0;//!<������֤��һ�ε���·�̲�ͻ�� ��Ϊ����ʱLastPosition�϶���0���ܻᵼ��һȦ�⻷
};
class cloud : public manager//!��չ:��̨��(6623)
{
	public:
		Motor_t *MotorType;        //!<���(���)����
		float RealAngle;           //!<���ݻ�е�Ǽ��������ʵ�Ƕ�
		//������
		int16_t TargetCurrent;     //!<���͸�����ĵ���ֵ
		int16_t RealCurrent;		   //!<ʵ��ת�ص���(������)
		//�ٶȻ�
		float RealSpeed;           //!<ʵ���ٶ�(������/������)
		float TargetSpeed;	       //!<Ŀ���ٶ�
		float *Gyro_RealSpeed;     //!<ָ�������Ƿ����ٶȵ�ָ��
		float Gyro_TargetSpeed;    //!<����������Ŀ��ת��
		//λ�û�
		int16_t RealPosition;	     //!<��ʵλ��(������)
		int16_t OriginalPosition;  //!<ԭʼλ��(������)��û�о���У��
		int16_t TargetPosition;	   //!<Ŀ��λ��
		float *Gyro_RealAngle;     //!<ָ�������Ƿ����Ƕȵ�ָ��
		float Gyro_TargetAngle;    //!<����������Ŀ��Ƕ�
	
		pid *PID_In;          //!<��е��PID�ٶȻ�
		pid *PID_Out;         //!<��е��PIDλ�û�
		pid *Gyro_PID_In;     //!<������PID�ٶȻ�
		pid *Gyro_PID_Out;    //!<������PIDλ�û�
	
		void Pid_Select(pid *PID_In_Select, pid *PID_Out_Select);//!<��е��pidѡ��
		void Gyro_Pid_Select(pid *Gyro_PID_In_Select, pid *Gyro_PID_Out_Select);//!<������pidѡ��
		void Speed_Set(float);     //!<�趨�ٶȣ�ͨ����е�ǵ���
		void Angle_Set(float);     //!<�趨�Ƕȣ�ͨ����е�ǵ���
		void Gyro_Speed_Set(float TargetSpeed);    //!<�趨�ٶȣ�ͨ�������ǵ���
		void Gyro_Angle_Set(float TargetPosition); //!<�趨�Ƕȣ�ͨ�������ǵ���
		void Limit(float _max,float _min);//!<���������λ
		void Gyro_Speed_Run(void);   //!PID���㺯����manager�����
		void Gyro_Position_Run(void);//!PID���㺯����manager�����
		cloud();//!<Ĭ�Ϲ��캯��
		cloud(uint8_t can_num,
						uint16_t _can_id,
						int16_t _CLOUD_STD,
						Motor_t *motor_type,
						pid *PID_I,
						pid *PID_O,
						pid *G_In,
						pid *G_Out,
						float *SpeedSource=NULL,
						float *PositionSource=NULL); //!<���캯����ָ��ָ��λ�û��ٶ�Դ
		virtual void Safe_Set(void);         //!<�趨������밲ȫģʽ�������͵���ֵ0
						
	protected:
		int16_t CLOUD_STD;          //!<����̨��ָ��ԭ��ʱ�ı�������ֵ
	
		int16_t LastTorque;     //!<�ϴ�ת��
		float LastSpeed;        //!<�ϴ��ٶ�(��е��)
		int16_t LastPosition;   //!<�ϴ�λ��(��е��)
		float Gyro_LastSpeed;   //!<�ϴ��ٶ�(������)
		float Gyro_LastPosition;//!<�ϴ�λ��(������)
		float max=99999999999;//!<�Ƕ����ֵ
		float min=-99999999999;//!<�Ƕ���Сֵ
		virtual void update(uint8_t Data[]); //!<ֱ��Data�����update����
		virtual void Handle(void);           //!<���ݴ������������ж�״̬������PID
		virtual void Speed_Run(void);        //!<ʹ���ٶȻ�ȷ������ ΪPID���㻷��
		virtual void Position_Run(void);     //!<ʹ��λ�û�ȷ������ ΪPID���㻷��
	
		virtual void InsertCurrent(void);    //!<������õĵ������б�������ͻ�������
};
class softcloud : public cloud///��·����̨�� for 6020
{
	public:
		float TargetAngle;         //!<���ݻ�е�Ǽ��������ʵ�Ƕ�
		int32_t Soft_RealPosition=0;//!<����ʵ·�̣�����ʵ����������̨ת����Ȧ��
		int32_t Soft_TargetPosition;//!<��Ŀ��·�̣�ʵ������Ϊ����Ҫת��Ȧ��
	
		softcloud():cloud(){}//�����ڹ���ն���ʱ���ɹ�
		softcloud(uint8_t can_num,
						uint16_t _can_id,
						int16_t _CLOUD_STD,
						Motor_t *motor_type,
						pid *PID_I,
						pid *PID_O,
						pid *G_In,
						pid *G_Out,
						float *SpeedSource=NULL,
						float *PositionSource=NULL)
				:cloud(can_num, _can_id, _CLOUD_STD, motor_type, PID_I, PID_O, G_In, G_Out, SpeedSource, PositionSource),SOFTCLOUD_STD(_CLOUD_STD){}//!<���캯����ָ��ָ��λ�û��ٶ�Դ
		void Angle_Set(float);//!<����·��Ŀ��Ƕ�
	private:
		virtual void Handle(void);           //!<���ݴ������������ж�״̬������PID
		virtual void update(uint8_t Data[]);//!<ֱ��Data�����update����
		virtual void Position_Run(void);
		int16_t SOFTCLOUD_STD;          //!<����̨��ָ��ԭ��ʱ�ı�������ֵ
		uint8_t running_flag=0;//!<������֤��һ�ε���·�̲�ͻ�� ��Ϊ����ʱLastPosition�϶���0���ܻᵼ��һȦ�⻷
//		float max=99999999999;//!<�Ƕ����ֵ
//		float min=-99999999999;//!<�Ƕ���Сֵ
};
/** 
* @brief  ���̵Ŀ������ͣ������ʿ���
* @par ��־ 
*   2019��3��9��16:43:30 WMD ��Ϊ��Ҫ���ٶȻ��͵�����֮����빦�����Ƶ����������������������޸���һ�¿�ܣ���������Ƕ��������ͣ�����ͨ������Ͳ��漰������
*/
class chassis
{
	public:
		float extra_power = 0;//���û�������������Ķ��⹦��
		pid *Pid_extra_power;//���û�������������⹦�ʵ�pid��ָ��
	
		static chassis* point;//!<ָ��ǰ�������ĵ��̣�һ������ֻ����һ�����̵Ĵ���  ��ָ�������պ��йܴ���
		//pid *Turn_PID;  //!<ת��PID
		void Run(void); //!<ȱʡ���� ���ϴε�ģʽ���Ƶ���
		void Run(float Vx, float Vy, float Omega);//!<���ٶȿ��Ƶ���
		void Safe(void);     //!ֹͣ����
		softmotor *Motor[4]; //!<�ĸ����ӵĵ������
		chassis(uint8_t can_num, 
							uint16_t First_can_id, 
							Motor_t *motor_type, 
							pid *speed_pid, 
							//pid* turnpid=NULL, 
							pid *current_pid=NULL, 
							int16_t *CurrentSource=NULL,
							pid *extra_power_pid=NULL);//!<ֱ�ӿ��Ƶ��̵Ĺ��캯��
		void Handle(void);//!<����CANSend�йܵĵ��̴�����
	private:
		RunState_t RunState;//!<��ǰ����״̬
		pid *Pid_spe[4];//!<ָ���ĸ�����ٶȻ�pid��ָ��
		pid *Pid_current[4];//!<ָ���ĸ����������pid��ָ��
		int16_t *CurrentSource[4];//!<ָ�������ʵ����Դ
		float Last_Vx=0, Last_Vy=0, Last_Omega=0;//!<֮ǰ��ֵ������ȱʡ����ʱ��ʹ��
}; 
class chassiscontrol//���̿�����for��̨
{
	public:
		CAN_HandleTypeDef* Canhandle;
		uint16_t Chassis_ID;
		chassiscontrol(CAN_HandleTypeDef* canhandle, uint16_t chassis_id);//���캯��
		void Run(void); //!<ȱʡ���� ���ϴε�ģʽ���Ƶ���
		void Run(int16_t Vx, int16_t Vy, int16_t Omega, uint8_t Mode, 
			uint8_t All_flags = 0);
		void Safe(void);     //!���̰�ȫģʽ
	private:
		int16_t Last_Vx=0, Last_Vy=0, Last_Omega=0;//֮ǰ��ֵ������ȱʡ����ʱ��ʹ��
		uint8_t Last_Mode=22;
};

#endif
