/**
* @file     app_AmmoFeed.hpp
* @brief    ͨ�ò��������Դ�ļ�
* @details  
* @author    ThunderDoge, Asn
* @date      2020.5
* @version  1.0.7
* @copyright  RM2020��� 
* @par ��־
*		v1.0.0  2019/11/29  ʵ�ֻ�������\n
*		v1.0.1  2019/12/6   Asn�����˲��ּ򻯣�ʹ������ʹ��\n
*		v1.0.2  2019/12/13  �����˲��ֶ�����������Լ�һЩ�﷨���󡣲�����ת������Blocked_Reaction�Դ����������գ�����PID�������\n
*		v1.0.3  2019/12.27  ������ת�ж��߼�����ͳһ���ⲿcansend������Ȼ�����������ʹ����淶\n
*		v1.0.4  2019/12/31  ��ԭ����Pr_Handle������ֲ��Handle�У�ʹSet��ģʽ֮��ֻ��Ҫ����manager::CANSend()�ͺ���\n
*		v1.0.5  2020/1/21   ������ȫģʽ��bug��������ר��ֹͣģʽ��������trig�궨��\n
*		v1.0.6	2020/2/24	���ӣ���ÿ��trig������Դ���ֵ��0��Set_Step���������ⲿֱ�����ò���
*       v1.0.7  2020/5/22   ���� last_mode�����ʹ��staticʹ�ö��ammofeed��������ŵ����� 
*/ 
#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

#define DBUS_RANGE    0 //1��dbusͨ��ֵ��Χ���������364~1684
						//0��dbusͨ��ֵ��Χ���������0~660


/** 
* @brief ����ִ�еĺ���
*/
void AmmoFeed::Handle(void)
{
	if(feed_mode != last_mode)
	{
		soft_target_angle = SoftAngle;//ģʽ�л�ʱ�Ƕ�ͬ��
	}
	if(block!=NULL && RunState!=Stop)
	{
		block->Block_Check();
		if(!Blocked_Reaction())//��תʱ���ж�ת����
		{
			switch(feed_mode)
			{
				case AMMOFEED_FREE_FIRE:
					Free_Fire();
					break;
				case AMMOFEED_BURST:
					Burst();
					break;
				case AMMOFEED_FREE_ONCE:
					Free_Once();
					break;
				default:
					Safe_Set();
					break;
			}
		}
	}
	last_mode = feed_mode;
}


/**
* @brief ��ת������
* @retval 0 �����˳�
* @retval ���� �쳣�˳�
*/
uint8_t AmmoFeed::Blocked_Reaction(void)
{
	if(block == NULL)
		return 0;
	if(block->IsBlock)
	{
		if( !is_block_in_handle )	//��ת��δ����
		{	//������ת�������
            dual_block_cnt ++;
			soft_target_angle = SoftAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//�趨��ת�Ƕ�
			Angle_Set(soft_target_angle);
			block->Clear_BlockFlag();	//�����ת��־
			is_block_in_handle = 1;	//�������־
		}
		else //�ڴ����Զ�ת
		{
            block_cnt ++;
			is_block_in_handle = 0;	//ǿ�ƻص�����
			block->Clear_BlockFlag();
		}
	}
	else
	{
		if(is_block_in_handle)	//δ��ת���ڴ���
		{
			if( fabs(SoftAngle - soft_target_angle) <=3 )	
			{
				is_block_in_handle = 0;	//�ﵽ�ǶȺ󷵻�����
			}
			else
			{
				Angle_Set(soft_target_angle);	//��������
			}
		}
		else	//δ��ת��������ɣ�ֱ�ӷ���
		{	return 0; }
	}
	return 1;
}

/** 
* @brief ���������еĺ���
*/
void AmmoFeed::Step_Run(void)
{
	static int16_t Step_Overflow;
	static uint32_t rammerStepTime = 0;
	if(( (HAL_GetTick() - rammerStepTime) > ramming_discrete_delay ) ) 
	{
		if(rammer_step_left > 0)//������ڲ���
		{
			soft_target_angle += ( SIGN(rammer_direction) * 360.0f/feeder_division) ;		//�趨��ת�Ƕȣ��������û��趨�ı��� rammer_direction ����
			rammer_step_left-- ;
			rammerStepTime = HAL_GetTick();
		}
		else if(rammer_step_left == 0)//�������Ϊ0�����е������İ���������ĽǶ�
		{
			Step_Overflow = (int16_t)floorf((soft_target_angle - SoftAngle)/(360.0f/feeder_division)/SIGN(rammer_direction));
			if(Step_Overflow<=0)Step_Overflow = 0;
			soft_target_angle -= ( SIGN(rammer_direction) * 360.0f/feeder_division)*Step_Overflow;
		}
	}
	Angle_Set(soft_target_angle);
	Position_Run();
}



/****************************ģʽ���ú���*********************************/

/**
* @brief ���ɿ���ģʽ���ú���
* @param FreeSpeed �����ٶ�
*/
void AmmoFeed::Free_Fire_Set(int32_t FreeSpeed)
{
	feed_mode = AMMOFEED_FREE_FIRE;
	RunState = Speed_Ctl;
  TargetSpeed = FreeSpeed;
}

/**
* @brief N����ģʽ���ú���
* @param DiscreDelay ÿ��һ�����ʱ��
* @param trig N�����Ĵ�������
*/
void AmmoFeed::Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t* trig)
{
	feed_mode = AMMOFEED_BURST;
	RunState = Position_Ctl;//ֻ������ȫģʽ�л���ָʾ����
	burst_shoot_cnt = ShootCnt;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}

/**
* @brief ��������ģʽ���ú���
* @param DiscreDelay ÿ��һ�����ʱ��
* @param trig �����Ĵ������� 
*/
void AmmoFeed::Free_Once_Set(int32_t DiscreDelay,int16_t* trig)
{
	feed_mode = AMMOFEED_FREE_ONCE;
	RunState = Position_Ctl;//ֻ������ȫģʽ�л���ָʾ����
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/**
* @brief Ŀ��Ƕ����ú���
* @details ��softmotorһ����ֻ��ȥ����RunState���ã���ΪRunState���ñ������˲������û��ӿڴ�
*/
void AmmoFeed::Angle_Set(float Target_Angle)
{
	if(Target_Angle>max)Target_Angle=max;//��Ŀ��Ƕ��޷�
	if(Target_Angle<min)Target_Angle=min;
	Target_Angle=Target_Angle*MotorType->Reduction_ratio/360;//�õ��������ǰĿ��Ȧ��
	TargetPosition=(Target_Angle-(int32_t)Target_Angle)*MotorType->max_mechanical_position;//С������ ����ǰ��Ȧλ��
	Soft_TargetPosition=(int32_t)Target_Angle;//�������� ������ǰȦ��
}
/**
* @brief ר��ֹͣģʽ
*/
void AmmoFeed::Stop_Set(void)
{
	if(block!=NULL)block->IsBlock=0;//ȥ����ת��־���������߼�����Ȼ��Ϊ�Ƕ�ת
	feed_mode = AMMOFEED_STOP;
	RunState = Stop;
	rammer_step_left = 0;
	soft_target_angle = SoftAngle;
	Angle_Set(soft_target_angle);
}

/*************************************************************************/



/****************************ģʽ���к���*********************************/
/** 
* @brief ���ɿ���ģʽ
*/
void AmmoFeed::Free_Fire(void)
{
	soft_target_angle = SoftAngle;//�˴�Ϊ���࣬��ֹģʽ�л���ת
	Speed_Run();
}


/** 
* @brief ��������ģʽ
*/
void AmmoFeed::Free_Once(void)
{
	static uint8_t act_flag=0;
	static uint32_t act_time_stamp;
	static uint8_t once_flag = 0;

#if DBUS_RANGE
	static uint16_t trig_set = 1224;
#else 
	static uint16_t trig_set = 200;
#endif

	if( *trigger>trig_set )
	{
		if( act_flag == 0 )
		{
			act_flag= 1;
			act_time_stamp = HAL_GetTick();//�����жϵ�����������
		}
		*trigger = 0;		//�������
	}
	else
	{	
		act_flag = 0;	
	}
	
	if(act_flag)
	{
		if((HAL_GetTick()-act_time_stamp)>free_once_trig_time)//���ﴥ��ʱ��
		{
			rammer_step_left++;			
		}
		else
		{
			if(once_flag == 0)
			{
				once_flag = 1;
			}
		}
	}
	else
	{
		if(once_flag == 1)
		{
			rammer_step_left = 1;//���η��䣬�͸���һ������
			once_flag = 0;
		}
		else
		{
			rammer_step_left = 0;//������ʱ�Ѳ�������
		}
	}
	Step_Run();//���������в���
}


/** 
* @brief N����ģʽ
*/
void AmmoFeed::Burst(void)
{
	static uint8_t burst_flag =0;	//����burst��־λ

#if DBUS_RANGE
	static uint16_t trig_set = 1224;
#else 
	static uint16_t trig_set = 200;
#endif

	if( *trigger>trig_set )	//���������
	{
		if(burst_flag == 0)	//����־λδ��
		{	
			rammer_step_left = burst_shoot_cnt;
			burst_flag = 1;	//����־������burst
		}
		*trigger = 0;		//�������
	}
	else
	{
		burst_flag = 0;
	}
	Step_Run();
}


/*****************************************************************************/




#undef SIGN
