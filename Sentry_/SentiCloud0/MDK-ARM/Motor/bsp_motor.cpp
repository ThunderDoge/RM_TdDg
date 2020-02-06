/** 
	* ��ʵ���˸о��Ǹ�����ţ��DJI�ؼ�������
	* @details  
	* ��ʼ�����������õ���Ķ��� CAN���ջص��Ϸ���CANUpdate() �����ڵ���CANSend() 
	* ʹ��:������Ĳ�������ֱ���� 
	* @author      WMD,Onion rain
	* @par Copyright (c):  
	*       WMD,Onion rain
	* @par ��־
	*		V6.0  ��motor�������������ݸ��ºͷ��Ͳ�����Ϊmanager�࣬��motor��Ϊmanager���࣬cloud��Ҳ��Ϊmanager���� 
	*		V6.1  ��̨�������λ�ò��ȡ�ٶȽ���ֵ(Ȼ����������) 
	*		V6.2  motor Angle_Set�βθ�Ϊfloat�ͣ���λΪ�� 
	*		V6.3  ����Motor_t�ṹ�壬��¼���(���)��������ʼ�������ʱ��ָ�봫������ 
	*		V6.4  pid���������ࣺ������pid 
	*		V6.5  ����softcloud�࣬6020���ר�� 
	*		V6.6  motor�༰�������ѡ������pid 
	*		V6.7a Motor_t�������ӿ��ٳ�ʼ���б� 
	*		V6.7b motor��cloud������RealAngle������ʾ�������λ�û���õ��ĽǶȣ���λΪ�㣬�ڸ������ڵ�update���� 
	*		V6.8a ����CANSend�����������޶�Position_Run()��Speed_Run()��ʹ��Handle()�������й������߼�
	*		V6.8b ����block_type�࣬�����޶���softmotor���ͣ���ӵ�и��Զ����ķ�ʽ
	*		V6.9a pid��Dout��Ϊ���D��ʹ��Dout_Accumulative�����ۼ�
	*		V6.9b ��̨��ʼ��ʱ����⻷pid������ʹ�������ǽ��ٶ���Ϊ�⻷΢��ֵ
	*		V6.9c �޸���softcloud��Angle_Set��е��ģʽ����λ���תͷ��bug
	*		V6.9d �����˵��̿�����chassiscontrol��������̨����̲�����ͨ��
	*		V6.10 �޸���pid����start_time���»����쳣��bug
	*		V6.10a �����޸���pid����start_time���»����쳣��bug
	*		V6.11  �޸���Block_Type���º��ܹ���������
	*		V6.12w  �Ƴ�������Դ��ĵ�������ͬʱ�ڵ����������ṩ�˵��������ƣ����ṩ�˹��ʿ���ϵͳ��ʹ��ʱ��Ҫ��CarDrv_Config.hpp����� \
						 ��������LIMIT_P�궨�壬 ����Ҫ���ƹ��ʵĻ����ṩ����������.ͬʱ�����@ref manager::UserProcess()�м䴦���������� \n
						 �û���pid���к���ǰ����Զ��崦���߼���ͬʱ�������û����Ѷ���ĵ����Ϊһ���Ƕ������ͨ������ @ref manager::cooperative 
						 ���趨�ò�����@ref manager::CANSend() �������Զ����иõ����pid����Ҫ�û���@ref manager::UserProcess()���д���pid.
	*		V6.12y  �޸��˷�����pid��һ���߼�����
	*		V6.13	���̿�����(for��̨)����һ����ͷ�ͳ������ݿ��أ����ݵ�����
<<<<<<< HEAD
	*		V6.14 �޸���chassis�಻���������������޹��ʵ�ʱ����������������
=======
	*		V6.14	motor�༰������LastUpdateTime�������ǰ�����������ͬ�������Լ��������֮��ͨ���жϵ����
	
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<RM2019��RM2020�ֽ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	* @author  Evan-GH
	*		V7.0 	�޸�pid::run���㺯��������T�������Ļ���ʱ�������
	*   V7.1 	�޸���̨��������ǽǶ����ô������㺯������err��Ϊ���������룬���ٳ���50�ˣ���������֮ǰд�Ĳ�������Ҫ���µ�����
	*		V7.2 	�޸���̨��Ļ�е�ǽǶ����ô������㺯�����޸�errΪ���������Ķ�Ŀ��ֵ�Ĵ��룬��Ӧ֮ǰ�����Ĳ���
	*		V7.2a ��pid::run���㺯������һ�μ����Ը��£���Ӧû�������ַ���Ĳ���
	*		V7.2b ʹ�û�е��ģʽ����Yaw���ʱ�������޷��˷�8192��0��ͻ�䣬����RM2019�����ϵĴ����softcloud::Angle_Set��softcloud::Position_Run
						��������������һ���޸ģ��Ѿ�����
	*		V7.3	�޸ĵ����࣬���ϱ�־λ
	*		V7.3a	����RM2019����������뻺��������һЩ��������������������
	*		V7.4	���ݴ���淶�޸���һ���ļ����������ڲ����������Ա�϶࣬�����ڲ������޸ģ������������Bug������淶�վ����Ƕ�C++��������
						Car_Driver.cpp -> bsp_motor.cpp			Car_Driver.hpp -> bsp_motor.hpp		universe.cpp -> bsp_universe.cpp
						CarDrv_config.hpp -> bsp_car_config.hpp  ע��ֻ���޸����ļ������ڲ�������û���������޸ģ�����ͨ�õģ�ʹ�÷������䣡
	*		V7.4a	�޸����ļ�������ϵ�����ڵ���⴦��Bsp�㣬���ٵ���app_math����ĺ������ļ��ֲ��������
	*		V7.5	�޸��˵�����߼���ʱ����ֵ10ms��50ms
	*		V7.5a	����ToolMan ThunderDoge�������C++�����������޸ģ��޸�֮ǰgitһ�±�ƽ��
						����GM6020���ֲᣬ6020�����ID���Դﵽ0x20B�����Դ˴��޸ĸ����˵���б������������͵���󳤶ȣ�������֧�ֵ�0x20B��
						�򵥲��Ժ�о������������ģ�����can_code�ı�ʾ����Ϊ��λ��ʾcan���������β����ʾID-1
	*		V7.5b ����ToolMan ThunderDoge�������C++�����������޸ģ���softmotor������³�Ա SoftAngle��RealAngle���ں�motor����㷽��һ�£���Χ
						0~360��SoftAngle��ΧΪ�����������
	*		V7.5c ����ToolMan ThunderDoge�������C++�����������޸ģ��޸���motor�������Ա������LastCurrentEncoder -> LastRealCurrent
						CurrentEncoder->RealCurrent
	*		V7.5d ����ToolMan ThunderDoge�������C++�����������޸ģ�����ԭʼλ����Ϣ��ԱOriginalPosition�������û�о���CLOUD_STDУ��
						����̨��������λ����������cloud������������cloud�������������λ�õı����ͳ�Աmax��min ������ֵֻ��ͨ��Limit������������
	*		V7.6	��pid������ȷ�˻���ʱ���΢��ʱ�䣬�����������ʱ��Ĭ��Ϊ1.�������������������Խ��е�����,���֣�΢���������������Լ�������ʱ�����
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
*/  
#include "bsp_motor.hpp"
#include "bsp_can.hpp"
#include <string.h>
#include <math.h>

#define WEAK __attribute__((weak)) //ʹ��WEAK�����Ƿ������������ع��ض�����
#define	ABS(x)   ((x)>0?(x):-(x))
using namespace std;
/** 
*   @brief ����������
*         Bit [0]:���״̬����Ԥ��
*/
uint8_t Error_Code=0x00;

////*******************************************��ͳPID��*************************************************************************////
/** 
	* @brief  ��ͳpid���캯��
	* @param [in]   P ����ϵ��
	* @param [in]	 I ����ϵ��
	* @param [in]	 D ΢��ϵ��
	* @param [in]	 IMax �����޷�
	* @param [in]	 PIDMax ����޷�
	* @param [in]	 I_Time ����ʱ��
	*	@param [in]	 D_Time ����ʱ��
	* @param [in]	 I_Limited ���л��ֵ��ڵ������������
	* @par ��־ 
	*
*/
WEAK pid::pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time, uint16_t D_Time,uint16_t I_Limited)
{
	this->P=P;
	this->I=I;
	this->D=D;
	this->IMax=IMax;
	this->PIDMax=PIDMax;
	this->I_Time=I_Time;
	this->D_Time=D_Time;
	this->I_Limited=I_Limited;
}
/** 
	* @brief  ������pid���캯��
	* @param [in]   Kp ��(ap,ap+bp),err=0ʱKp��С,bpΪ�仯����,cp����Kp�仯����,ż������������
	* @param [in]   Ki ��(0,ai),err=0ʱki���,ci����Ki�仯����,ż������������
	* @param [in]   Kd ��(ad,ad+bd),err=0ʱKd=ad+bd/(1+cd),dd����Kd�仯����,�����ݼ�
	* @param [in]	 IMax �����޷�
	* @param [in]	 PIDMax ����޷�
	* @param [in]	 T ����ʱ��
	* @param [in]	 I_Limited ���л��ֵ��ڵ������������
	* @par ��־ 
	*
*/
pid::pid(float ap, float bp, float cp,
				 float ai,           float ci,
				 float ad, float bd, float cd, float dd,
				 float IMax, float PIDMax, uint16_t I_Time, uint16_t D_Time, uint16_t I_Limited)
{
	this->ap = ap; this->bp = bp; this->cp = cp;
	this->ai = ai;                this->ci = ci;
	this->ad = ad; this->bd = bd; this->cd = cd; this->dd = dd;
	this->IMax=IMax;
	this->PIDMax=PIDMax;
	this->I_Time=I_Time;
	this->D_Time=D_Time;
	this->I_Limited=I_Limited;
}
/** 
	* @brief  pid���к���
	* @param [in]   err ����pid������� 
	* @retval  pid��������� 
	* @par ��־ 
*				2019��12��1��15:00:00 �Ƴ�����ʱ�䲻���õļ����ԸĶ�������΢��ʱ��
*/
WEAK float pid::pid_run(float err)
{
	CurrentError = err;
	Pout = CurrentError*P;
	
	//���ַ���
	if(HAL_GetTick() - I_start_time >= I_Time)//����ﵽ��ʱ������Ļ�����л������
	{
		if(ABS(CurrentError) < I_Limited)//����С���������ʱ����I����
			Iout += I	*	CurrentError;
		else 
			Iout=0;					//���������߻�����0
		I_start_time = HAL_GetTick();//���¶�����ֿ�ʼʱ��
	}
	
	if(Custom_Diff!=NULL)//�����Զ���΢������
		Dout_Accumulative=(*Custom_Diff)*D;
	else
		Dout_Accumulative=(CurrentError - LastError)*D;
	
	if(HAL_GetTick() - D_start_time > D_Time)//����ﵽ��ʱ������Ļ������΢�����
	{
		Dout=Dout_Accumulative;
		Dout_Accumulative = 0;
		D_start_time = HAL_GetTick();//���¶���΢�ֿ�ʼʱ��
	}
	
	if(Iout	>=	IMax)Iout=IMax;
	if((Iout)	<=	-(IMax))Iout=-(IMax);	//�����޷�
	
	PIDout = Pout + Iout + Dout;				//Pid�������
	if(PIDout	>=	PIDMax)PIDout = PIDMax;
	if(PIDout	<=	-(PIDMax))PIDout = -(PIDMax); //����޷�
	
	LastError=CurrentError;
	return PIDout;
}
/** 
	* @brief  ������pid���к���
	* @param [in]   err ����pid������� 
	* @retval  pid��������� 
	* @par ��־ 
	*
*/
float pid::nonlinear_pid_run(float err)
{
	P = ap + bp * (1 - sech(cp * err));
	I = ai * sech(ci * err);
	
	//������pid������ڲ������ߡ���kd����Ϊ�����ݼ�������ò��ֻ֧��һ�ֱ仯���򣬴˴�����������kd(err)��err>0
	if(err > 0)
		D = ad + bd / (1 + cd * exp(dd * err));
	else 
		D = ad + bd / (1 + cd * exp(dd * (-err)));
	
	return pid_run(err);
}
float pid::sech(float in)
{
	return 1/cosh(in);
}
////*******************************************block_type��******************************************************************////
/** 
	* @brief  ��ʼ����ת���
	* @param [in]	Limit ������ת��ֵ
	* @param [in]	 time ��ת����ʱ��
	* @param [in]	 err_num �ڶ��ٷ�Χ����Ϊ�Ƕ�ת
	* @par ��־ 
    *       2019��1��19��10:41:56 �ع�block_typeʹ�ڲ�����࣬�ӿڸ����㣬���ܸ�ȫ��
	*		2019��1��24��14:46:09 ���ĸýӿڱ�����ʽ
	*
*/
void block_type::Block_Init(uint16_t Limit, uint16_t time, float err_num)
{
	block_Current_Limit=Limit;
	block_Time_Limit=time;
	block_err_num=err_num;
}
/** 
	* @brief  ��ת��� �ú������ɵ���й�����ִ��
	* @retval  1 ��ת
	* @retval  0 δ��ת
	* @par ��־ 
	*
*/
void block_type::Block_Check(void)
{
	if(ABS(Current)>this->block_Current_Limit)//����������ֵ���ж�Ϊ��ת
	{
		if(block_flag==0)//û�еǼǶ�ת
		{
			block_flag=1;
			block_Angle=RealAngle;//�Ǽǵ�ǰ��ת��λ��
			block_time=HAL_GetTick();//�Ǽǵ�ǰ��תʱ��
		}else//�Ѿ��ǼǶ�ת �ж��Ƿ����
		{
			if(ABS(block_Angle-RealAngle) < block_err_num)//�ǼǶ�תʱ�����˿̼���δת����ֵ�Ƕ�
			{
				if(HAL_GetTick()-block_time > block_Time_Limit)//��תʱ�������ֵʱ��
				{
					IsBlock=1;
					return;
				}
			}else block_flag=0;//�Ѿ��仯�� ���ٶ�ת��
		}
	}else block_flag=0;
}
/** 
* @brief  �����ת��־λ
* @details ���¸�λ��ת�������������ת��־
* @retval  OK  �ɹ� 
* @retval  ERROR   ����  
* @par ��־ 
*
*/
void block_type::Clear_BlockFlag(void)
{
	block_flag=0;
	IsBlock=0;
}
////*******************************************manager��*********************************************************************////
/** 
	* @brief �ܼ���ľ�̬����,���ڴ洢�����Ӧid�Ĵ�����ֵ,��ǰ����ֵ���ȼ�,�Լ��䱾��ָ��
*/
CAN_HandleTypeDef* manager::CanHandle1;
CAN_HandleTypeDef* manager::CanHandle2;
manager* manager::CAN1MotorList[11]={NULL};
int16_t manager::CAN1CurrentList[11]={0};
int16_t manager::CAN1_OnlineID;
manager* manager::CAN2MotorList[11]={NULL};
int16_t manager::CAN2CurrentList[11]={0};
int16_t manager::CAN2_OnlineID;
/** 
	* @brief  ȷ��ѡ��ʹ�õ�CAN��
	* @param [in]   canhandle1 CAN�ľ�� һ��Ϊhcan1��hcan2
	* @param [in]   canhandle2 CAN�ľ�� һ��Ϊhcan1��hcan2
	* @par ��־ 
	*       2018��10��31��17:11:18 ������־
*/
void manager::CANSelect(CAN_HandleTypeDef* canhandle1, CAN_HandleTypeDef* canhandle2)
{
	CanHandle1=canhandle1;
	CanHandle2=canhandle2;
}
/** 
	* @brief  ��⵱ǰ����Ƿ�����
	* @retval  1  ���ˣ�������
	* @retval  0  ��������
	* @par ��־ 
*				2019��11��30��  �޸�can_code��ʾ�����ͼ�ⷽ��
*/
uint8_t manager::Is_Offline(void)
{
	if(can_code/100==1)//ΪCAN1���
			return !(CAN1_OnlineID & (0x01<<(can_code%100)));
	else//ΪCAN2���
			return !(CAN2_OnlineID & (0x01<<(can_code%100)));
}
/**
	* @brief  ������ݸ����ܹܳ���,�������е�������ݽ���
	* @param [in]   _hcan ������жϵ�CAN
	* @param [in]   RxHead �������ݸ�ʽ�ṹ��ָ��
	* @param [in]   Data ��������ָ��
	* @retval  OK  �ɹ�
	* @retval  ERROR   ����
	* @par ��־ 
	*      2018��10��10��17:24:53 �����¹̼����֧��
*			 2019��11��30�� 16:32:15 �Ե��ID����һ����չ�Ķ�
*/
WEAK void manager::CANUpdate(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef* RxHead,uint8_t* Data)
 {
	uint16_t id=RxHead->StdId;
	if(_hcan==CanHandle1)
	{
		if(id<=0x20B && id>=0x201)//�Ǳ�׼�Ĵ�����֡
			if(CAN1MotorList[id-0x201]!=NULL)
				CAN1MotorList[id-0x201]->update(Data);
	}
	if(_hcan==CanHandle2)
		if(id<=0x20B && id>=0x201)//�Ǳ�׼�Ĵ�����֡
			if(CAN2MotorList[id-0x201]!=NULL)
				CAN2MotorList[id-0x201]->update(Data);
}
/** 
* @brief  CAN�����ϵ��Զ������ݴ�����
* @par ��־ 
*       2019��3��8��16:12:26 wmd �ú���������
*/
WEAK void manager::UserProcess(void)
{
    //�ú�������ִ���κβ�������Ҫ��PID���꣬����֮ǰִ�д���Ļ������Լ����ļ���д�ú�������
    UNUSED(UserProcess);
}
/** 
	* @brief  CAN�����ܹ�.���е�CAN���������������д�������
	* @retval  0 �ɹ�
	* @retval  ��һλ��1 CAN1 0x200����ʧ��
	* @retval  �ڶ�λ��1 CAN1 0x1ff����ʧ��
	* @retval  ����λ��1 CAN1 0x2ff����ʧ��
	* @retval  ����λ��1 CAN2 0x200����ʧ��
	* @retval  ����λ��1 CAN2 0x1ff����ʧ��
	* @retval  ����λ��1 CAN2 0x2ff����ʧ��
	* @par ��־
	*       2018��10��31��16:46:48 �����߼�����
	*       2018��11��1��20:54:09  �������ߵ���������ߵ�������͵Ĺ����ڴ˴�ʵ��
	*       2019��1��19��14:34:55  �����߼���һ���Ż���ÿһ�������������λ�ú��ٶ����֣� �����������ӹ��ܣ�������Ҫ��CANSend����ǿ�� 
  *       2019��3��14��15:09:13 �����˺������͵�����������͵������CANSend���Զ�����Ҫ��������Ӧ��UserHandle�д���
	*				2019��11��30��15:01:01 ����GM6020��ID���ܳ���0x208�����
*/
uint8_t manager::CANSend(void)
{
	if(CanHandle1!=NULL)//����CAN1����
	{
		for(uint8_t i=0;i<11;i++)//��11�����������ѯ
		{
			if(CAN1MotorList[i]!=NULL)//�������������
			{
				if(CAN1MotorList[i]->cooperative==0)CAN1MotorList[i]->Handle();//�������Э���͵������ִ���䵥������
				if(HAL_GetTick() - CAN1MotorList[i]->LastUpdateTime > 50)//�������������Ѿ�����50msû�и�����
				{
					CAN1_OnlineID &= ~(0x01<<i);//����Ӧonlineλ��0
				}else
				{//����Ϊ�������
					CAN1_OnlineID |=0x01<<i;//����Ӧonlineλ��1
				}
			}
		}
	}
	if(CanHandle2!=NULL)//����CAN2����
	{
		for(uint8_t i=0;i<11;i++)//��11�����������ѯ
		{
			if(CAN2MotorList[i]!=NULL)//�������������
			{
				if(CAN2MotorList[i]->cooperative==0)CAN2MotorList[i]->Handle();//�������Э���͵������ִ���䵥������
				if(HAL_GetTick() - CAN2MotorList[i]->LastUpdateTime > 50)//�������������Ѿ�����50msû�и�����
				{
					CAN2_OnlineID &= ~(0x01<<i);//����Ӧonlineλ��0
				}else
				{//����Ϊ�������
					CAN2_OnlineID |=0x01<<i;//����Ӧonlineλ��1
				}
			}
		}
	}
	if(chassis::point!=NULL)chassis::point->Handle();//������ڵ��̵Ļ�����е��̵Ĺ��ʿ��ƴ���
	UserProcess();//�����û����Զ������ݴ���
	//����Ϊ���ʹ���
	uint8_t check=0;
	uint8_t result=0;
	//CAN1��������
	if((CAN1_OnlineID&0x0f)!=0)//�жϸ�������û�����ߵ��
		check=bsp_can_Sendmessage(CanHandle1,0x200,(int16_t*)CAN1CurrentList);
	if(check)result|=0x01;
	if((CAN1_OnlineID&0xf0)!=0)//�жϸ�������û�����ߵ��
		bsp_can_Sendmessage(CanHandle1,0x1ff,(int16_t*)&CAN1CurrentList[4]);
	if(check)result|=0x02;
	if((CAN1_OnlineID&0x700)!=0)//�жϸ�������û�����ߵ��
		bsp_can_Sendmessage(CanHandle1,0x2ff,(int16_t*)&CAN1CurrentList[8]);
	if(check)result|=0x04;
	
	//CAN2��������
	if((CAN2_OnlineID&0x0f)!=0)//�жϸ�������û�����ߵ��
		bsp_can_Sendmessage(CanHandle2,0x200,(int16_t*)CAN2CurrentList);
	if(check)result|=0x08;
	if((CAN2_OnlineID&0xf0)!=0)//�жϸ�������û�����ߵ��
		bsp_can_Sendmessage(CanHandle2,0x1ff,(int16_t*)&CAN2CurrentList[4]);
	if(check)result|=0x10;
	if((CAN2_OnlineID&0x700)!=0)//�жϸ�������û�����ߵ��
		bsp_can_Sendmessage(CanHandle2,0x2ff,(int16_t*)&CAN2CurrentList[8]);
	if(check)result|=0x20;
	memset(CAN1CurrentList,0,22);
	memset(CAN2CurrentList,0,22);
	return result;
}
WEAK void manager::Speed_F_Set(float f)///�趨ǰ����
{
	Speed_F=f;
}
////*******************************************motor��***********************************************************************////
/** 
	* @brief  ����๹������
	* @param [in]  can_num	�ڼ���CAN ֻ��������1��2
	* @param [in]  _can_id		�õ����CAN_ID,��:0x201
	* @param [in]  *motor_type ������ͽṹ��ָ��
	* @param [in]  _PID_In	�ڻ�PID ����pid����ָ��
	* @param [in]	 _PID_Out �⻷PID	����pid����ָ�� 
	* @param [in]	 _PID_Current ������PID	����pid����ָ�� 
	* @param [in]	 CurrentSource �����������������
	* @par ��־ 
*				2019��11��30��  �޸�can_code��ʾ��ʽ
*/
WEAK motor::motor(const uint8_t can_num, const uint16_t _can_id, Motor_t *motor_type, pid *_PID_In, pid *_PID_Out)
	:MotorType(motor_type), PID_In(_PID_In), PID_Out(_PID_Out)
{
	//��motor�������������Լ���ָ��
	if(can_num==1)CAN1MotorList[_can_id-0x201]=this;
	if(can_num==2)CAN2MotorList[_can_id-0x201]=this;
	can_code=can_num*100+_can_id-0x201;
}
/** 
	* @brief  ����жϸ������ݵĺ���
	* @param [in]   Data[] �ж�ʱ�ӵ�������
	* @par ��־ 
	*       2018��10��31��19:19:56 ��������update����Ϊһ��update����
*				2019��12��1��3:32:56 �޸ĳ�Ա�� LastCurrentEncoder -> LastRealCurrent    CurrentEncoder->RealCurrent
*/
WEAK void motor::update(uint8_t Data[])
{
	LastSpeed = RealSpeed;
	LastPosition = RealPosition;
	LastRealCurrent = RealCurrent;//����ת�ص���
	RealPosition = Data[0]<<8 | Data[1];
	RealSpeed = Data[2]<<8 | Data[3];
	RealCurrent = Data[4]<<8 | Data[5];
	if(RealCurrent != LastRealCurrent)//ǰ��ת�ص�����ͬ��������Ч����
		LastUpdateTime = HAL_GetTick();//���±��ε�����������µ�ʱ��
	RealAngle = RealPosition*360.f/MotorType->max_mechanical_position;//���ݻ�е�Ǽ��������ʵ�Ƕ�
}
/** 
* @brief  ����������ݴ���ĺ��� �ú�������CAN����ǰִ�У�һ�������ж�����״̬������PID�������
* @par ��־ 
*
*/
WEAK void motor::Handle(void)
{
	if(block!=NULL && RunState!=Stop)
	{
		block->Block_Check();
	}
	switch(RunState)
	{
		case Speed_Ctl://��������ٶ�̬����
			Speed_Run();
			break;
		case Position_Ctl://·��
			Position_Run();
			break;
		case Stop://ֹͣ״̬
			Safe_Set();
			break;
		default:
			Error_Code|=0x01;//���ɴ�����
			Safe_Set();
			break;
	}
}
/** 
	* @brief  �Ըõ��ִ�а�ȫָ��
	* @par ��־ 
	*
*/
WEAK void motor::Safe_Set(void)
{
	if(block!=NULL)block->IsBlock=0;//ȥ����ת��־���������߼�����Ȼ��Ϊ�Ƕ�ת
	RunState = Stop;
	TargetCurrent = 0;
	InsertCurrent();
}
/** 
	* @brief  �����λ�ø���ֵ
	* @note �ú���Ϊ���õ��ת��ת��һȦ�ڵ�λ��
	* @param [in]   Target_Angle Ŀ��Ƕ�
	* @par ��־ V1.0 �����ú���
	*           V1.1 �βθ�Ϊfloat�ͣ���ʾ�Ƕȣ���λ��
	*
*/
WEAK void motor::Angle_Set(float Target_Angle)
{
  RunState = Position_Ctl;
	TargetPosition = Target_Angle/360.f * MotorType->max_mechanical_position;
}
/** �������Ŀ���ٶ�
	* @param [in]   Speed Ŀ���ٶ�
	* @par ��־
	*       2018��10��31��17:31:27 �����ú���
	*
*/
WEAK void motor::Speed_Set(int16_t Speed)
{
	RunState=Speed_Ctl;
  TargetSpeed=Speed;
}
/** 
	* @brief  �趨�����λ��PID
	* @warning �ú�����λ�û���·���ۼ�ЧӦ��ֻ�����ڲ�����һȦ����ͨ��̨���
	* @retval  Current �õ����ʱӦ�÷��͵ĵ���ֵ
	* @note �˺�����motor::CANSend�й�
	* @par ��־ 
	*       2018��10��31��17:01:51 ������Angle_Set���ָ�Ϊ˽�к�����ͳһ����
	*
*/
WEAK void motor::Position_Run(void)
{
	if(PID_Out==NULL)while(1);//���û���⻷pid�Ļ�ֱ�ӱ�¶����
	int16_t err=	TargetPosition	-	RealPosition;//��ʱ�������Ϊ·�̻����
	err=PID_Out->pid_run(err);//������ʱerrΪĿ���ٶ�
	TargetSpeed=err;
	Speed_Run();//��Ŀ���ٶȷŽ��ٶȿ�����
}
/** 
	* @brief  �趨����ٶ�PID
	* @note �ú����ɿ��ڲ��йܴ���
	* @par ��־ 
	*       2018��10��31��17:34:04 �����ú���
    *       2019��3��9��16:20:28 ɾ���˶�Current_Run()�ļ������������θĶ����ܹ����¼���
*/
WEAK void motor::Speed_Run(void)
{
	TargetCurrent = PID_In->pid_run(this->TargetSpeed - RealSpeed);
	if(Speed_F!=0)//ʹ����ǰ������
	{
		//ǰ�������
		Speed_LPF = LPF_NUM*(RealSpeed-LastSpeed) + (1-LPF_NUM)*(Speed_LPF);//�ٶ�ǰ����ͨ�˲���
		TargetCurrent = TargetCurrent + Speed_LPF * Speed_F;//�˴��ڶ���Ϊ�ٶȻ�ǰ��
	}
    InsertCurrent();
}
/** 
	* @brief  ������PID
	* @note �ú����ɿ��ڲ��йܴ���
	* @par ��־ 
	*       2019��1��14�� �����ú���
	*
*/
//WEAK void motor::Current_Run(void)
//{
//	if(PID_Current==NULL)while(1);//���е������������޵�����pid����¶����
//	if(RealSpeed<0)*RealCurrent *= -1;//������������
//	CurrentSend = PID_Current->pid_run(this->TargetCurrent - *RealCurrent);
//	InsertCurrent();
//}
/** 
	* @brief  �������ȼ���Ҫ���͵ĵ���ֵ�����б��еȴ�����
	* @par ��־ 
	*
*/
WEAK void motor::InsertCurrent(void)
{
	if(can_code/100==1)
		CAN1CurrentList[can_code%100] = TargetCurrent;
	if(can_code/100==2)
		CAN2CurrentList[can_code%100] = TargetCurrent;
}
/** 
	* @brief  �Ըõ�����ö�ת���
	* @param [in]   Limit ������ת��ֵ
	* @param [in]	 time ��ת����ʱ��
	* @param [in]	 err_num �ڶ��ٷ�Χ����Ϊ�Ƕ�ת
	* @retval  0  �ɹ�
	* @retval  -1 ���� �����Ƕ�ջ�ռ䲻��
	* @par ��־
	*
*/
int8_t motor::Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num)
{
	if(block==NULL)
	{
		//�����ڴ�
		block=new block_type(TargetCurrent, RealAngle);//ע�� ����ĵ�������ֵ��softmotor��һ��
		if(block==NULL)return -1;
	}
	block->Block_Init(Limit, time, err_num);
	return 0;
}
////*******************************************softmotor��*******************************************************************////
/** 
	* @brief  ���µ��ֵ ��������·�̵Ľ���
	* @retval  none
	* @par ��־ 
	*
*/
WEAK void softmotor::update(uint8_t Data[])
{
	motor::update(Data);//���ø���motor��ͨ��update����
	if(running_flag==0)
	{
		LastPosition=RealPosition;
		running_flag=1;
	}
	if(RealPosition	-	LastPosition	>	4096)//Ȧ���ۼ�
		Soft_RealPosition--;
	else if(RealPosition	-	LastPosition	<-4096)//Ȧ���ۼ�
		Soft_RealPosition++;
//	RealAngle = /*Ȧ����Ӧ�Ƕ�*/(Soft_RealPosition)/(1.0*MotorType->Reduction_ratio)*360 \
//							+ /*��Ȧ�ڽǶ�*/1.0f*RealPosition / (8192 * MotorType->Reduction_ratio )*360;//ת��Ϊ�Ƕ� �������������softangle
	RealAngle = RealPosition*360.f/MotorType->max_mechanical_position;//���ݻ�е�Ǽ��������ʵ�Ƕ�
	SoftAngle = /*Ȧ����Ӧ�Ƕ�*/(Soft_RealPosition)/(1.0*MotorType->Reduction_ratio)*360 \
							+ /*��Ȧ�ڽǶ�*/1.0f*RealPosition / (8192 * MotorType->Reduction_ratio )*360;//ת��Ϊ�Ƕ�
}
///����·�̽������Ʒ�Χ
WEAK void softmotor::Limit(float _max, float _min)
{
	max=_max;
	min=_min;
}
/** 
	* @brief   �в�����λ�û�ȷ�� ��ʹ���ƶ���ָ��λ��
	* @param [in]   Target_Angle Ŀ��Ƕ�
	* @retval  ����ֵ
	* @par ��־ 
	*       V1.0 2018��10��31��19:43:18 ��PID���Ƴ����˳��� �ú���ֻ�ܸ����趨Ŀ��ֵ��
	*       V1.1 �βα�ʾ�Ƕȣ���λ��
*/
WEAK void softmotor::Angle_Set(float Target_Angle)
{
  RunState=Position_Ctl;
	if(Target_Angle>max)Target_Angle=max;//��Ŀ��Ƕ��޷�
	if(Target_Angle<min)Target_Angle=min;
	Target_Angle=Target_Angle*MotorType->Reduction_ratio/360;//�õ��������ǰĿ��Ȧ��
	TargetPosition=(Target_Angle-(int32_t)Target_Angle)*MotorType->max_mechanical_position;//С������ ����ǰ��Ȧλ��
	Soft_TargetPosition=(int32_t)Target_Angle;//�������� ������ǰȦ��
}
/** 
	* @brief   �޲�����λ�û�+�ٶȻ�����
	* @retval  ����ֵ
	* @note   �ú�����д���ຯ������cansendѭ�����ò��ɸ���
	* @par ��־ 
	*       2018��10��31��17:13:46 ��������������motor::CANSend()�й�ִ��
	*       2018��11��3��11:32:35  ������·�̻�У�黷��
*/
WEAK void softmotor::Position_Run(void)
{
  if(PID_Out==NULL)while(1);//��¶������û���⻷������λ��PID?
	int32_t err=0;
	err=TargetPosition-RealPosition;     //�õ���Ȧ���
	err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//����Ȧ��λ�����
	TargetSpeed = PID_Out->pid_run(err);//λ�û��õ�Ŀ���ٶ�
	TargetCurrent = PID_In->pid_run(TargetSpeed-RealSpeed);
	InsertCurrent();
}
/** 
	* @brief  �Ըõ�����ö�ת���
	* @param [in]   Limit ������ת��ֵ
	* @param [in]	 time ��ת����ʱ��
	* @param [in]	 err_num �ڶ��ٷ�Χ����Ϊ�Ƕ�ת
	* @retval  0  �ɹ�
	* @retval  -1 ���� �����Ƕ�ջ�ռ䲻��
	* @par ��־
	*
*/
int8_t softmotor::Enable_Block(uint16_t Limit, uint16_t time, uint16_t err_num)
{
	if(block==NULL)
	{
		//�����ڴ�
		block=new block_type(TargetCurrent, SoftAngle);
		if(block==NULL)return -1;
	}
	block->Block_Init(Limit, time, err_num);
	return 0;
}
////*******************************************Cloud��***********************************************************************////
/** 
	* @brief  ��̨������캯�� \n
	* 				������ģʽ�����������Ǻ���̨һ���ʱ������� �����ʹ��������ģʽ ���Ը���Чֵ
	* @note   ��ƫ�Ǽ��� ��ƫ������ \n
	*				  һ��ע�⣡����������������������������������������������������������������̨Yaw�������°�װ����֮�����һ��Ҫ�ģ�������
	* @param [in]   can_num	�ڼ���CAN ֻ��������1��2
	* @param [in]   _can_id		�õ����CAN_ID,��:0x201
	* @param [in]   _CLOUD_STD ����̨��ָ��ԭ��ʱ�ı�������ֵ
	* @param [in]   *motor_type ������ͽṹ��ָ��
	* @param [in]   PID_I		��е�ǿ����ڻ�
	* @param [in]   PID_O		��е�ǿ����⻷
	* @param [in]   G_In		�����ǿ����ڻ�
	* @param [in]   G_Out		�����ǿ����⻷
	* @param [in]   SpeedSource	    �������ٶ�����Դ
	* @param [in]   PositionSource	������λ������Դ
	* @par ��־ 
	*
*/
WEAK cloud::cloud(uint8_t can_num, uint16_t _can_id, int16_t _CLOUD_STD, Motor_t *motor_type, pid* _PID_In, pid* _PID_Out, pid* G_In, pid* G_Out, float *SpeedSource, float *PositionSource)//!<���캯��������ָ��λ�û��ٶ�Դ
	:MotorType(motor_type), Gyro_RealSpeed(SpeedSource), Gyro_RealAngle(PositionSource)
{
	//��motor�������������Լ���ָ��
	PID_In  = _PID_In;
	PID_Out = _PID_Out;
	if(can_num==1)CAN1MotorList[_can_id-0x201]=this;
	if(can_num==2)CAN2MotorList[_can_id-0x201]=this;
	can_code=can_num*100+_can_id-0x201;
	
	CLOUD_STD=_CLOUD_STD;
	Gyro_PID_In=G_In;
	Gyro_PID_Out=G_Out;
	
	if(Gyro_PID_Out != NULL)//��������pid�⻷����ʱ���⻷��΢��ֵ����Ϊ�������ǽ��ٶ�
		Gyro_PID_Out->Custom_Diff = Gyro_RealSpeed;
}
/** 
	* @brief  cloud���͵ĸ��º���
	* @param [in]   Data[] �жϴ���ĸ�������
	* @note 1.�ú������RealPosition���л���STD_YAW�ı�׼��λ \n
	*       2.�ú���������ͨ���������ٶ�ֵ��6623�ǲ������ٶȵģ��ٶ�ȡ�����Ƿ����ٶ� \n
	* @par ��־ 
*				2019��12��1��3:44:20 ����ԭʼλ����Ϣ��ԱOriginalPosition�������û�о���CLOUD_STDУ��
*/
WEAK void cloud::update(uint8_t Data[])
{
	LastPosition=RealPosition;
	LastTorque=RealCurrent;
	LastSpeed=RealSpeed;
	RealPosition=Data[0]<<8 | Data[1];  //ʵ��λ��
	OriginalPosition = RealPosition; //ԭʼ����
	//��е�ǵ���ʵֵ��Ҫ��һ��ƽ�� �Ա�֤������0λ�������0λ��
	if(RealPosition-CLOUD_STD<-4096)RealPosition = RealPosition-CLOUD_STD+8192;
	else if(RealPosition-CLOUD_STD>4096)RealPosition = RealPosition-CLOUD_STD-8192;
	else RealPosition = RealPosition-CLOUD_STD;
	RealCurrent=Data[2]<<8 | Data[3];   //ʵ��ת�ص���
	if(Gyro_RealSpeed != NULL) RealSpeed = *Gyro_RealSpeed;     //���ⲿ���������ٶ�ȡ�������ٶ�
	//��е�������ٶ��𵴺����ض���1ms������������ٶ�Ҳ��С��һ�˲�û�ˣ�����
	else RealSpeed = (LastPosition - RealPosition + LastSpeed)/2; //���ⲿ�����������λ��ƫ������ȡ���ٶ�,������ֵ�˲�
	LastUpdateTime=HAL_GetTick();  //���±��ε�����������µ�ʱ��
	RealAngle = RealPosition*360.f / MotorType->max_mechanical_position;//ת��Ϊ�Ƕ�
}
/** 
* @brief  ����������ݴ���ĺ��� �ú�������CAN����ǰִ�У�һ�������ж�����״̬������PID�������
* @par ��־ 
*
*/
WEAK void cloud::Handle(void)
{
	switch(RunState)
	{
		case Speed_Ctl://��������ٶ�̬����
			Speed_Run();
			break;
		case Position_Ctl://·��
			Position_Run();
			break;
		case Gyro_Position_Ctl://�������ض�ģʽ ע���״ֻ̬����cloud����ӵ��
			Gyro_Position_Run();
			break;
		case Gyro_Speed_Ctl://�������ض�ģʽ ע���״ֻ̬����cloud����ӵ��
			Gyro_Speed_Run();
			break;
		case Stop://ֹͣ״̬
			Safe_Set();
			break;
		default:
			Error_Code|=0x01;//���ɴ�����
			Safe_Set();
			break;
	}
}
/** 
	* @brief  �����л���е��pid
	* @par ��־ 2018.12.11����
	*
*/
WEAK void cloud::Pid_Select(pid *PID_In_Select, pid *PID_Out_Select)
{
	PID_In = PID_In_Select;
	PID_Out = PID_Out_Select;
}
/** 
	* @brief  �����л�������pid
	* @par ��־ 2018.12.11����
	*
*/
WEAK void cloud::Gyro_Pid_Select(pid *Gyro_PID_In_Select, pid *Gyro_PID_Out_Select)
{
	Gyro_PID_In = Gyro_PID_In_Select;
	Gyro_PID_Out = Gyro_PID_Out_Select;
}
/** 
	* @brief  �����λ
	* @par ��־ 
*				2019��12��1��4:00:00 �������λ����������cloud������������cloud�������������λ�õĳ�Ա
*/
WEAK void cloud::Limit(float _max, float _min)
{
	max=_max;
	min=_min;
}
/** �������Ŀ���ٶ�(��е��)
	* @param [in]   Speed Ŀ���ٶ�
	* @par ��־
	*
*/
WEAK void cloud::Speed_Set(float Speed)
{
	RunState=Speed_Ctl;
	TargetSpeed=Speed;
}
/** 
	* @brief  �����λ�ø���ֵ(��е��)
	* @param [in]   Target_Angle Ŀ��Ƕ�
	* @par ��־ 
	*       V1.1 �βα�ʾ�Ƕȣ���λ��
	*
*/
WEAK void cloud::Angle_Set(float Target_Angle)
{
  RunState=Position_Ctl;
	TargetPosition=Target_Angle*MotorType->max_mechanical_position/360;
	if(TargetPosition >= max)
		TargetPosition = max;
	if(TargetPosition <= min)
		TargetPosition = min;
	//TargetPosition=Target_Angle*MotorType->max_mechanical_position; //�������޸ģ��趨�Ƕȵ�ʱ�����Ҫ��Ҫ���ɻ�е����ֵ�������Ӧ��ϵ��������
	//����Ч������Ӧ�ò��Ǵ����������⣬���⻷΢������û�����úã���Ҫ�ⲿ�Լ�������
}
/** 
	* @brief  ͨ��������ģʽ�趨�ٶ�
	* @param [in]   TargetSpeed Ŀ���ٶ�ֵ���ñ���Ӧ�����������ǵ��ٶ�ֵ 
	* @par ��־ 
*/
WEAK void cloud::Gyro_Speed_Set(float TargetSpeed)
{
	RunState=Gyro_Speed_Ctl;
	Gyro_TargetSpeed=TargetSpeed;
}
/** 
	* @brief  ͨ��������ģʽ�趨�Ƕ�
	* @param [in]   Target_Angle Ŀ��Ƕ�ֵ���ñ���Ӧ�����������ǵĽǶ�ֵ 
	* @par ��־ 
	*       V1.0 2018��10��31��17:22:06 ���ĺ����ӿ� �ú����������趨������ ����������
	*
*/
WEAK void cloud::Gyro_Angle_Set(float Target_Angle)
{
	RunState=Gyro_Position_Ctl;
	Gyro_TargetAngle=Target_Angle;
	if(Target_Angle >= max)
		Target_Angle = max;
	if(Target_Angle <= min)
		Target_Angle = min;
}
/** 
	* @brief  �趨����ٶ�PID(��е��)
	* @note �ú����ɿ��ڲ��йܴ���
	* @par ��־
	     2018��10��31��17:34:04 �����ú���
	     2018/12/18���ӷ�����pid����
	*
*/
void cloud::Speed_Run(void)
{
	if(PID_In->ap==0 && PID_In->bp==0)//�ڻ�pidΪ��ͳpid
		TargetCurrent = PID_In->pid_run(TargetSpeed - RealSpeed);             //��ͳpid����
	else TargetCurrent = PID_In->nonlinear_pid_run(TargetSpeed - RealSpeed);//������pid����
	if(Speed_F!=0)//ʹ����ǰ������
	{
		//ǰ�������
		Speed_LPF = LPF_NUM*(RealSpeed-LastSpeed) + (1-LPF_NUM)*(Speed_LPF);//�ٶ�ǰ����ͨ�˲���
		TargetCurrent = TargetCurrent + Speed_LPF * Speed_F;                //�˴��ڶ���Ϊ�ٶȻ�ǰ��
	}
	InsertCurrent();
}
/** 
	* @brief  �趨�����λ��PID(��е��)
	* @warning �ú�����λ�û���·���ۼ�ЧӦ��ֻ�����ڲ�����һȦ����ͨ��̨���
	* @retval  Current �õ����ʱӦ�÷��͵ĵ���ֵ
	* @note �˺�����manager::CANSend�й�
	* @par ��־ 
          2018/12/18���ӷ�����pid����
	*
*/
WEAK void cloud::Position_Run(void)
{
	if(PID_Out==NULL)while(1);                    //���û���⻷pid�Ļ�ֱ�ӱ�¶����
	//int16_t err = TargetPosition	-	RealPosition; //��ʱ�������Ϊ·�̻����
	float err = TargetPosition	-	RealPosition;		//��ʱ�������Ϊ·�̻����
	if(PID_Out->ap==0 && PID_Out->bp==0)          //�⻷pidΪ��ͳpid
		err=PID_Out->pid_run(err);                  //��ͳpid������ʱerrΪĿ���ٶ�
	else err = PID_Out->nonlinear_pid_run(err);   //������pid������ʱerrΪĿ���ٶ�
	TargetSpeed=err;
	Speed_Run();//��Ŀ���ٶȷŽ��ٶȿ�����
}
/** 
	* @brief  �趨��̨����ٶȵ�PID����(������)
	* @note �ú�����CAN_Send()�йܴ���
	* @par ��־ 
	       2018��11��27��19:44:19 �½��ú���
	       2018/12/18���ӷ�����pid����
*/
WEAK void cloud::Gyro_Speed_Run(void)
{
	while(this->Gyro_RealSpeed == NULL);//δָ���������ٶ�����Դ��¶����
	
	if(Gyro_PID_In->ap==0 && Gyro_PID_In->bp==0)//�ڻ�pidΪ��ͳpid
		TargetCurrent = Gyro_PID_In->pid_run(Gyro_TargetSpeed - *Gyro_RealSpeed);             //��ͳpid����
	else TargetCurrent = Gyro_PID_In->nonlinear_pid_run(Gyro_TargetSpeed - *Gyro_RealSpeed);//������pid����
	if(Speed_F!=0)//ʹ����ǰ������
	{
		//ǰ�������
		Speed_LPF = LPF_NUM*(*Gyro_RealSpeed-Gyro_LastSpeed) + (1-LPF_NUM)*(Speed_LPF);//�ٶ�ǰ����ͨ�˲���
		TargetCurrent = TargetCurrent + Speed_LPF * Speed_F;//�˴��ڶ���Ϊ�ٶȻ�ǰ��
	}
	InsertCurrent();
}
/** 
	* @brief  �趨��̨����Ƕȵ�PID����(������)
	* @note �ú�����CAN_Send()�йܴ���
	* @par ��־
          2018/12/18���ӷ�����pid����
*/
WEAK void cloud::Gyro_Position_Run(void)
{
	if(Gyro_PID_Out==NULL)while(1);//���û���⻷pid�Ļ�ֱ�ӱ�¶����
	while(Gyro_RealAngle == NULL);//δָ��������λ������Դ,��¶����
	//int16_t err = 50 * (Gyro_TargetAngle-*Gyro_RealAngle);//ȷ�����
	float err = Gyro_TargetAngle-*Gyro_RealAngle;//ȷ�����
	if(Gyro_PID_Out->ap==0 && Gyro_PID_Out->bp==0)  //�⻷pidΪ��ͳpid
		err=Gyro_PID_Out->pid_run(err);               //��ͳpid������ʱerrΪĿ���ٶ�
	else err = Gyro_PID_Out->nonlinear_pid_run(err);//������pid������ʱerrΪĿ���ٶ�
	Gyro_TargetSpeed = err;
	Gyro_Speed_Run();//��Ŀ���ٶȷŽ��ٶȿ�����
}
/** 
	* @brief  �������ȼ���Ҫ���͵ĵ���ֵ�����б��еȴ�����
	* @par ��־ 
	*
*/
WEAK void cloud::InsertCurrent(void)
{
	if(can_code/100==1)
		CAN1CurrentList[can_code%100] = TargetCurrent;
	if(can_code/100==2)
		CAN2CurrentList[can_code%100] = TargetCurrent;
}
/** 
	* @brief  �Ըõ��ִ�а�ȫָ��
	* @par ��־ 
	*
*/
WEAK void cloud::Safe_Set(void)
{
	RunState = Stop;
	TargetCurrent=0;
	InsertCurrent();
}
////*******************************************softcloud��***********************************************************************////
/** 
	* @brief  ���µ��ֵ ��������·�̵Ľ���
	* @retval  none
	* @par ��־ 
	*					2019��12��1��3:44:20 ����ԭʼλ����Ϣ��ԱOriginalPosition�������û�о���CLOUD_STDУ��
*/
WEAK void softcloud::update(uint8_t Data[])
{
	LastPosition=RealPosition;
	LastTorque=RealCurrent;
	LastSpeed=RealSpeed;
	RealPosition=Data[0]<<8 | Data[1];  //ʵ��λ��
	OriginalPosition = RealPosition; //ԭʼλ�����ݣ�û�о���У��
	//��е�ǵ���ʵֵ��Ҫ��һ��ƽ�� �Ա�֤������0λ�������0λ��
	if(RealPosition-CLOUD_STD<(-MotorType->max_mechanical_position/2))
		RealPosition = RealPosition - CLOUD_STD + MotorType->max_mechanical_position;
	else if(RealPosition-CLOUD_STD>(MotorType->max_mechanical_position/2))
		RealPosition = RealPosition - CLOUD_STD - MotorType->max_mechanical_position;
	else RealPosition = RealPosition-CLOUD_STD;
	RealCurrent=Data[4]<<8 | Data[5];   //ʵ��ת�ص���
	if(Gyro_RealSpeed != NULL) RealSpeed = *Gyro_RealSpeed;     //���ⲿ���������ٶ�ȡ�������ٶ�
	else RealSpeed = (int16_t)(Data[2]<<8 | Data[3]);
	
	LastUpdateTime=HAL_GetTick();  //���±��ε�����������µ�ʱ��
	if(running_flag==0)//�����һ��
	{
		LastPosition=RealPosition;
		running_flag=1;
	}
	if((RealPosition-LastPosition) > (MotorType->max_mechanical_position/2))//Ȧ���ۼ�
		Soft_RealPosition--;
	else if((RealPosition-LastPosition) < (-MotorType->max_mechanical_position/2))//Ȧ���ۼ�
		Soft_RealPosition++;
	RealAngle = /*Ȧ����Ӧ�Ƕ�*/(Soft_RealPosition)/(1.0*MotorType->Reduction_ratio)*360 \
							+ /*��Ȧ�ڽǶ�*/1.0f*RealPosition / (MotorType->max_mechanical_position * MotorType->Reduction_ratio )*360;//ת��Ϊ�Ƕ�
}
/** 
* @brief  ����������ݴ���ĺ��� �ú�������CAN����ǰִ�У�һ�������ж�����״̬������PID�������
* @par ��־ 
*
*/
WEAK void softcloud::Handle(void)
{
	switch(RunState)
	{
		case Speed_Ctl://��������ٶ�̬����
			Speed_Run();
			break;
		case Position_Ctl://·��
			Position_Run();
			break;
		case Gyro_Position_Ctl://�������ض�ģʽ ע���״ֻ̬����cloud����ӵ��
			Gyro_Position_Run();
			break;
		case Gyro_Speed_Ctl://�������ض�ģʽ ע���״ֻ̬����cloud����ӵ��
			Gyro_Speed_Run();
			break;
		case Stop://ֹͣ״̬
			Safe_Set();
			break;
		default:
			Error_Code|=0x01;//���ɴ�����
			Safe_Set();
			break;
	}
}
/** 
	* @brief   �в�����λ�û�ȷ�� ��ʹ���ƶ���ָ��λ��
	* @param [in]   Target_Angle Ŀ��Ƕ�
	* @retval  ����ֵ
	* @par ��־ 
*				2019��12��1��3:55:20 ��Ŀ��ǶȽ������޷�
*/
WEAK void softcloud::Angle_Set(float Target_Angle)
{
  RunState=Position_Ctl;
//	if(Target_Angle>max)Target_Angle=max;//��Ŀ��Ƕ��޷�
//	if(Target_Angle<min)Target_Angle=min;
//	Target_Angle=Target_Angle*MotorType->Reduction_ratio/360;//�õ��������ǰĿ��Ȧ��
//	TargetPosition=(Target_Angle-(int32_t)Target_Angle)*MotorType->max_mechanical_position;//С������ ����ǰ��Ȧλ��
////	Soft_TargetPosition=(int32_t)Target_Angle/360.f;//�������� ������ǰȦ��
//	Soft_TargetPosition=Soft_RealPosition;//����360��̨��е��ģʽ����Ҫ��Ȧ���˴�Ŀ��Ȧ����Ϊ��ǰȦ������ֹ��̨ת����Ȧ����̨����ת
//	if(TargetPosition>MotorType->max_mechanical_position/2)
//	{
//		TargetPosition -= MotorType->max_mechanical_position;
//	}
	if(Target_Angle >= max)
		Target_Angle = max;
	if(Target_Angle <= min)
		Target_Angle = min;  //�ԽǶ��趨ֵ�����޷�
	TargetAngle = Target_Angle;
}
/** 
	* @brief   �޲�����λ�û�+�ٶȻ�����
	* @retval  ����ֵ
	* @note   �ú�����д���ຯ������cansendѭ�����ò��ɸ���
	* @par ��־
*/
WEAK void softcloud::Position_Run(void)
{
  if(PID_Out==NULL)while(1);//��¶������û���⻷������λ��PID?
	int32_t err=0;
	err = (TargetAngle-RealAngle)*MotorType->max_mechanical_position/360;     //�õ����
	//err=TargetPosition-RealPosition;     //�õ���Ȧ���
	//err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//����Ȧ��λ�����
	if(PID_Out->ap==0 && PID_Out->bp==0)               //�⻷pidΪ��ͳpid
		TargetSpeed=PID_Out->pid_run(err);               //��ͳpid����õ�Ŀ���ٶ�
	else TargetSpeed = PID_Out->nonlinear_pid_run(err);//������pid�����õ�Ŀ���ٶ�
	if(PID_In->ap==0 && PID_In->bp==0)                 //�ڻ�pidΪ��ͳpid
		TargetCurrent=PID_In->pid_run(TargetSpeed-RealSpeed);                //��ͳpid����õ�Ŀ�����
	else TargetCurrent = PID_In->nonlinear_pid_run(TargetSpeed-RealSpeed); //������pid�����õ�Ŀ�����
	InsertCurrent();
}
chassis* chassis::point;
////*******************************************ȫ�µ�����********************************************************************////
/** 
	* @brief  ���̶��󹹽�����
	* @param [in]   can_num CAN��� ��ѡֵ1 2 
	* @param [in]   First_can_id ���̵���ĵ�һ��id�� ������0x201 �����Ϊ0x201,0x202,0x203,0x204
	* @param [in]   *motor_type ������ͽṹ��ָ��
	* @param [in]   speed_pid �ٶȻ�PIDָ��
  * @param [in]   current_pid ������PIDָ�� ����Ϊ�� Ϊ�����ʾ�ó����޹���
  * @param [in]   CurrentSource ����������Դ ����Ϊ�� ͬ��
  * @param [in]   extra_power_pid ���⹦��PIDָ�� ����Ϊ�� ͬ��
	* @retval  OK  �ɹ� 
	* @retval  ERROR   ����  
	* @par ��־ 
	*
*/
WEAK chassis::chassis(uint8_t can_num, uint16_t First_can_id, Motor_t *motor_type,pid *speed_pid, pid *current_pid, int16_t *CurrentSource, pid *extra_power_pid):Pid_extra_power(extra_power_pid)
{
	if(point==NULL)point=this;
	else while(1);//�ߵ�����˵����������2�������ϵĵ��̶����ˣ�
	//�����̵����Ӻ�pid��vector ���4��
	for(uint8_t i=0;i<4;i++)
	{
		Pid_spe[i]=new pid(*speed_pid);
		if(current_pid!=NULL)Pid_current[i]=new pid(*current_pid);
        else Pid_current[i]=0;
		if(CurrentSource!=NULL)this->CurrentSource[i]=CurrentSource+i;
        else this->CurrentSource[i]=NULL;
		Motor[i]=new softmotor(can_num, First_can_id+i, motor_type, Pid_spe[i], NULL);
		//Motor[i]->cooperative=1;//�����õ��Ϊ���̻�����һ���֣���ʹ�õ����ķ��ͺ���
	}
}
/** 
	* @brief  �����˶����ƺ���
	* @param [in]   Vx x���˶��ٶ�
	* @param [in]   Vy y���˶��ٶ�
	* @param [in]   Omega ��ת�ٶ�
	* @par ��־ 
	*
*/
WEAK void chassis::Run(float Vx, float Vy, float Omega)
{
	Last_Vx=Vx;
	Last_Vy=Vy;
	Last_Omega=Omega;

	float proportion, MaxSpeed,Buffer[4];
	int16_t Speed[4];
	uint8_t index;
	
	//���������ֵ��̵����˶�ѧģ��
	Buffer[0] = (Vx + Vy + Omega);
	Buffer[1] = (Vx - Vy - Omega);
	Buffer[2] = (Vx - Vy + Omega);
	Buffer[3] = (Vx + Vy - Omega);
	
	//���趨ֵ�е����ֵ
	for(index=0, MaxSpeed=0; index<4; index++)
	{
		if((Buffer[index]>0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
		{
			MaxSpeed = (Buffer[index]>0 ? Buffer[index] : -Buffer[index]);
		}
	}
	//���ٶ��趨ֵ����������������ٶȣ���ȱȼ�С�ٶ��趨ֵ
	if(ChassisMax < MaxSpeed)
	{
		proportion = (float)ChassisMax / MaxSpeed;
		Speed[0] = Buffer[0] * proportion;
		Speed[1] = -Buffer[1] * proportion;
		Speed[2] = Buffer[2] * proportion;
		Speed[3] = -Buffer[3] * proportion; 
	}
	else
	{
		Speed[0] =  Buffer[0];
		Speed[1] =  -Buffer[1];
		Speed[2] =  Buffer[2];
		Speed[3] =  -Buffer[3];
	}
	for(uint8_t i=0; i<4; i++)
	{
		Motor[i]->Speed_Set(Speed[i]);
	}
}
WEAK void chassis::Run(void) //����������Run������ԭ�ٶȽ���
{
	this->Run(Last_Vx, Last_Vy, Last_Omega);
}
WEAK void chassis::Safe()//�����ƶ�
{
	for(uint8_t i=0; i<4; i++)
		Motor[i]->Safe_Set();
}
/** 
* @brief  ���̵��йܴ������ ���鱣�ֺ�CANSendͬ����ִ�� ��Ҫ�ֶ���UserHandle�е���
* @par ��־ 
*   2019��3��9��16:48:07 �ú���������
*   2019��3��19��15:37:36 ��Ҫ��CarDriver_Config.hpp�����LIMIT_P�Ķ��壬����ᱨ��
*/
WEAK void chassis::Handle()
{
	//���׵��ȷ�ϳ����Ƿ��ڷ�ֹͣ״̬����Ҫ���ƹ���
	if(Motor[0]->RunState==0 || Pid_current[0]==NULL)return;
	//ע�⣺��Ĭ�ϴ�����Ϊ����֤�������ƵĹ��ܣ����ҵ�Դ��ѹΪ��������Ҫ���볬�����ݻ��Դ�仯�Ļ���Ҫ��д
	float calcPower = 0;
	uint8_t i;    
	for(i=0; i<4; i++)//�����ĸ����ӵĹ��ʼ�����Ҫ���޵Ĺ���
		calcPower += 24.0f*ABS(point->Motor[i]->TargetCurrent)/819.2f;
	
	if(calcPower>LIMIT_P)//���۹����Ѿ�����
		for(i=0; i<4; i++)
			Motor[i]->TargetCurrent *= LIMIT_P/calcPower;//��������ѹ��
	int8_t flag = 1;//������������־ 1Ϊ�� -1λ��
	for(i=0; i<4; i++)
	{
		if(Motor[i]->RealSpeed < 0)flag = -1;
		else flag = 1;
		Motor[i]->TargetCurrent = Pid_current[i]->pid_run(Motor[i]->TargetCurrent/819.2f*1000.0f-*CurrentSource[i]*flag);
		Motor[i]->InsertCurrent();
	}
}
////*******************************************���̿�����for��̨********************************************************************////
/** 
	* @brief  ���̿�������󹹽�����
	* @param [in]   can_num CAN��� ��ѡֵ1 2 
	* @param [in]   Chassis_id ����id
	* @retval  OK  �ɹ� 
	* @retval  ERROR   ����  
	* @par ��־ 
	*
*/
WEAK chassiscontrol::chassiscontrol(CAN_HandleTypeDef* canhandle, uint16_t chassis_id)
{
	Canhandle = canhandle;
	Chassis_ID = chassis_id;
}
/** 
	* @brief  �����˶����ƺ���
	* @param [in]   Vx x���˶��ٶ�,ע��˴��ٶ�Ϊint16����
	* @param [in]   Vy y���˶��ٶ�
	* @param [in]   Omega ��ת�ٶ�
	* @param [in]   Mode 	����ģʽ
	* @param [in]   Reverse ���̷��򿪹�
	* @param [in]   SuperCap �������ݿ���
	* @par ��־ 
	*
*/
WEAK void chassiscontrol::Run(int16_t Vx, int16_t Vy, int16_t Omega, uint8_t Mode, 
	uint8_t All_flags)
{
	Last_Vx=Vx;
	Last_Vy=Vy;
	Last_Omega=Omega;
	Last_Mode=Mode;
	
	int16_t sendbuf[4] = {0};
	sendbuf[0] = (Mode<<8) | (All_flags);
	sendbuf[1] = Vx;
	sendbuf[2] = Vy;
	sendbuf[3] = Omega;
	
	bsp_can_Sendmessage(Canhandle, Chassis_ID, sendbuf);
}
WEAK void chassiscontrol::Run(void) //����������Run������ԭ�ٶȽ���
{
	this->Run(Last_Vx, Last_Vy, Last_Omega, Last_Mode);
}
WEAK void chassiscontrol::Safe()//�����ƶ�
{
	Last_Mode = 0x00;
	int16_t sendbuf[4] = {0};
	sendbuf[0] = 0xff00 & (Last_Mode<<8);
	bsp_can_Sendmessage(Canhandle, Chassis_ID, sendbuf);
}

//ж�غ궨��
#undef ABS
