#include "app_chassis.h"

#define ABS(x)   ((x)>0?(x):-(x))
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))
#define ChassisMax 15000    //!<������е��̵�����ٶ�


const static uint8_t Skidstep_Up[2] = {15,15};		//���Ƽ��ټ��ٶȣ�������
const static uint8_t Skidstep_Down[2] = {25,25};	//���Ƽ����ټ��ٶ�[0]��ǰ���Vy��[1]������


void Chassis_Run(float Vx, float Vy, float Omega)
{
//	Last_Vx=Vx;
//	Last_Vy=Vy;
//	Last_Omega=Omega;

	float proportion, MaxSpeed,Buffer[4];
	int16_t Speed[4];
	uint8_t index;
	
	static float ABS_Out_vx, ABS_Out_vy;//ABS����ٶ�ֵ
	
	ABS_Out_vx += (ABS(Vx)-ABS(ABS_Out_vx))>0 ? Skidstep_Up[0]*SIGN(Vx-ABS_Out_vx) : Skidstep_Down[0]*SIGN(Vx-ABS_Out_vx);
	ABS_Out_vy += (ABS(Vy)-ABS(ABS_Out_vy))>0 ? Skidstep_Up[1]*SIGN(Vy-ABS_Out_vy) : Skidstep_Down[1]*SIGN(Vy-ABS_Out_vy);
	if(ABS(Vx - ABS_Out_vx) < 300)  ABS_Out_vx = Vx;
	if(ABS(Vy - ABS_Out_vy) < 300)  ABS_Out_vy = Vy;// Ҫ��Ȼ�Ӽ����������ᾲֹ�������в�

	//���������ֵ��̵����˶�ѧģ��
	Buffer[0] = (ABS_Out_vx + ABS_Out_vy + Omega);
	Buffer[1] = (ABS_Out_vx - ABS_Out_vy - Omega);
	Buffer[2] = (ABS_Out_vx - ABS_Out_vy + Omega);
	Buffer[3] = (ABS_Out_vx + ABS_Out_vy - Omega);
	
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
//	for(uint8_t i=0; i<4; i++)
//	{
//		Motor[i]->Speed_Set(Speed[i]);
//	}
	Motor_1_Data.TargetSpeed = Speed[0];
	Motor_2_Data.TargetSpeed = Speed[1];
	Motor_3_Data.TargetSpeed = Speed[2];
	Motor_4_Data.TargetSpeed = Speed[3];
}

/**
* @brief  ���̰�ȫģʽ
* @details  
* @param[in]  
* @retval  
*/
void Chassis_Stop(void)
{
//		CAN2_TxData16[0] = Motor_1_Data.TargetCurrent;
//		CAN2_TxData16[1] = Motor_2_Data.TargetCurrent;
//		CAN2_TxData16[2] = Motor_3_Data.TargetCurrent;
//		CAN2_TxData16[3] = Motor_4_Data.TargetCurrent;
	memset( CAN2_TxData16 , 0 , sizeof( int16_t )*4 );
	can_send_msg(&hcan2, 0x200, CAN2_TxData16); 
	osDelay(1);

}



