#include "app_chassis.h"

#define ABS(x)   ((x)>0?(x):-(x))
#define SIGN(x) ((x)>0?1:((x)<0?-1:0))
#define ChassisMax 15000    //!<电机库中底盘的最大速度


const static uint8_t Skidstep_Up[2] = {15,15};		//限制加速加速度，成正比
const static uint8_t Skidstep_Down[2] = {25,25};	//限制减速速加速度[0]是前后的Vy，[1]是左右


void Chassis_Run(float Vx, float Vy, float Omega)
{
//	Last_Vx=Vx;
//	Last_Vy=Vy;
//	Last_Omega=Omega;

	float proportion, MaxSpeed,Buffer[4];
	int16_t Speed[4];
	uint8_t index;
	
	static float ABS_Out_vx, ABS_Out_vy;//ABS输出速度值
	
	ABS_Out_vx += (ABS(Vx)-ABS(ABS_Out_vx))>0 ? Skidstep_Up[0]*SIGN(Vx-ABS_Out_vx) : Skidstep_Down[0]*SIGN(Vx-ABS_Out_vx);
	ABS_Out_vy += (ABS(Vy)-ABS(ABS_Out_vy))>0 ? Skidstep_Up[1]*SIGN(Vy-ABS_Out_vy) : Skidstep_Down[1]*SIGN(Vy-ABS_Out_vy);
	if(ABS(Vx - ABS_Out_vx) < 300)  ABS_Out_vx = Vx;
	if(ABS(Vy - ABS_Out_vy) < 300)  ABS_Out_vy = Vy;// 要不然加减不整，不会静止，或者有差

	//长方形麦轮底盘的逆运动学模型
	Buffer[0] = (ABS_Out_vx + ABS_Out_vy + Omega);
	Buffer[1] = (ABS_Out_vx - ABS_Out_vy - Omega);
	Buffer[2] = (ABS_Out_vx - ABS_Out_vy + Omega);
	Buffer[3] = (ABS_Out_vx + ABS_Out_vy - Omega);
	
	//求设定值中的最大值
	for(index=0, MaxSpeed=0; index<4; index++)
	{
		if((Buffer[index]>0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
		{
			MaxSpeed = (Buffer[index]>0 ? Buffer[index] : -Buffer[index]);
		}
	}
	//若速度设定值超过底盘允许最大速度，则等比减小速度设定值
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
* @brief  底盘安全模式
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



