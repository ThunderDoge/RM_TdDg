/**
  * @brief    ͨ�ò��������
  * @details  
  * @author   ThunderDoge & Asn
  * @date     2019/12/13
  * @version  v1.0.3 
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  * v1.0.0  2019/11/29  ʵ�ֻ�������
  * v1.0.1  2019/12/6   Asn�����˲��ּ򻯣�ʹ������ʹ�á�
  * v1.0.2  2019/12/13  �����˲��ֶ�����������Լ�һЩ�﷨���󡣲�����ת������Blocked_Reaction�Դ����������գ�����PID���������
	*	v1.0.3  2019/12.27  ������ת�ж��߼�����ͳһ���ⲿcansend������Ȼ�����������ʹ����淶
	* v1.0.4  2019/12/31  ��ԭ����Pr_Handle������ֲ��Handle�У�ʹSet��ģʽ֮��ֻ��Ҫ����manager::CANSend()�ͺ���
  */

#include "app_AmmoFeed.hpp"
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))

/** 
    * @brief ����ִ�еĺ���
*/
void AmmoFeed::Handle(void)
{
	static uint8_t last_mode;
	if(block!=NULL && RunState!=Stop)
	{
		block->Block_Check();
	}
	if(!Blocked_Reaction())//��תʱ���ж�ת����
	{
		if(feed_mode != last_mode)
		{
			soft_target_angle = SoftAngle;
		}
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
	last_mode = feed_mode;
}


/** 
* @brief  ��ת������
* @param[in]   �� 
* @retval   �Ƿ��ڴ����ס�ı�־λ 1�����ڴ���0��ʾû�п�����ת����û�ж�ת
* @par ��־ 
*
*/
uint8_t AmmoFeed::Blocked_Reaction(void)
{
	if(block == NULL)
		return 0;
	if(block->IsBlock)
	{
		if( !is_block_in_handle )	//��ת��δ����
		{	//������ת�������
			soft_target_angle = SoftAngle - SIGN(TargetSpeed) * rev_angle_when_blocked ;	//�趨��ת�Ƕ�
			Angle_Set(soft_target_angle);
			block->Clear_BlockFlag();	//�����ת��־
			is_block_in_handle = 1;	//�������־
		}
		else //�ڴ����Զ�ת
		{
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
* @brief  ���ɿ���ģʽ���ú���
* @param[in]   �����ٶ�
* @retval  
* @par ��־ 
*
*/
void AmmoFeed::Free_Fire_Set(int32_t FreeSpeed)
{
	feed_mode = AMMOFEED_FREE_FIRE;
	ramming_speed = FreeSpeed;
}
/** 
* @brief  N����ģʽ���ú���
* @param[in]   DiscreDelay ÿ��һ�����ʱ��   trig N�����Ĵ������� 
* @retval   
* @par ��־ 
*
*/
void AmmoFeed::Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig)
{
	feed_mode = AMMOFEED_BURST;
	burst_shoot_cnt = ShootCnt;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/** 
* @brief  ��������ģʽ���ú���
* @param[in]   DiscreDelay ÿ��һ�����ʱ��   trig �����Ĵ������� 
* @retval   
* @par ��־ 
*
*/
void AmmoFeed::Free_Once_Set(int32_t	DiscreDelay,int16_t trig)
{
	feed_mode = AMMOFEED_FREE_ONCE;
	ramming_discrete_delay = DiscreDelay;
	trigger = trig;
}
/*************************************************************************/



/****************************ģʽ���к���*********************************/
/** 
    * @brief ���ɿ���ģʽ
*/
void AmmoFeed::Free_Fire(void)
{
	Speed_Set(ramming_speed);
	soft_target_angle = RealAngle;
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

	if( trigger )
	{
		if( act_flag == 0 )
		{
			act_flag= 1;
			act_time_stamp = HAL_GetTick();
		}
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
	
	if( trigger )	//���������
	{
		if(burst_flag == 0)	//����־λδ��
		{	
			rammer_step_left = burst_shoot_cnt;
			burst_flag = 1;	//����־������burst
		}
	}
	else
	{
		burst_flag = 0;
	}
	Step_Run();
}


void AmmoFeed::Safe_Set(void){
	Speed_Set(0);
	rammer_step_left = 0;
	feed_mode = 0;
}
/*****************************************************************************/








#undef SIGN

