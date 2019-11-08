#include "app_cloud.h"

//云台PID全局变量，便于调试
PID_t PID_Cloud_Yaw, PID_Cloud_Yaw_Speed,  PID_Cloud_Pitch , PID_Cloud_Pitch_Speed;
int32_t pitch_et, pitch_ut, pitch_Ie, pitch_de, pitch_s_et, pitch_s_ut, pitch_s_Ie, pitch_s_de;

/**
* @brief  云台电机Yaw和Pitch，PID初始化
* @details  
* @param[in]  
* @retval  
*/

void Cloud_PID_Init(void)
{
	PID_Cloud_Pitch.IntegralNeighbourAbs = fabs(100.0F);	//手动设定积分区间
	PID_Init(&PID_Cloud_Pitch ,
	&Motor_Pitch_Data.TargetPosition ,
	&Motor_Pitch_Data.RealPosition ,	
	&Motor_Pitch_Data.TargetSpeed ,
	0 , 0 , 0 , 
	2000 , 30000);																		
	
	PID_Init(&PID_Cloud_Pitch_Speed , 
	& Motor_Pitch_Data.TargetSpeed ,
	& Motor_Pitch_Data.RealSpeed , 
	& Motor_Pitch_Data.TargetCurrent , 
	0 , 0 , 0 ,
	2000, 30000);
	
	PID_Init(&PID_Cloud_Yaw , 
	&Motor_Yaw_Data.TargetPosition , 
	&Motor_Yaw_Data.RealPosition , 
	&Motor_Yaw_Data.TargetSpeed , 
	0 , 0 , 0 ,
	2000 , 30000);
	
	PID_Init(&PID_Cloud_Yaw_Speed , 
	&Motor_Yaw_Data.TargetSpeed , 
	&Motor_Yaw_Data.RealSpeed	,
	&Motor_Yaw_Data.TargetCurrent	,
	0 , 0 , 0 ,
	2000 , 30000);
}

/**
* @brief  云台电机停止
* @details  
* @param[in]  
* @retval  
*/
void  Cloud_Stop(void)
{
	memset( CAN1_TxData16 , 0 , sizeof(int16_t)*4 );
	can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);
}

/** 
* @brief    云台以机械角PID
* @details  
* @author   
* @date      
* @version  
* @par Copyright (c):  RM2020
* @par 
*/
void Cloud_RunEncoder(uint16_t ecdr_pitch, uint16_t ecdr_yaw)
{
	//编码器反馈的串级PID
	Motor_Pitch_Data.TargetPosition = ecdr_pitch;
	Motor_Yaw_Data.TargetPosition		= ecdr_yaw;
	
	Motor_PID_Loop_Section(&PID_Cloud_Pitch);
	Motor_PID_Loop_Section(&PID_Cloud_Pitch_Speed);
	Motor_PID_Loop_Section(&PID_Cloud_Yaw);
	Motor_PID_Loop_Section(&PID_Cloud_Yaw_Speed);
	pitch_et = PID_Cloud_Pitch.e_t * 1000;
	pitch_ut = PID_Cloud_Pitch.u_t *1000;
	pitch_Ie = PID_Cloud_Pitch.integ_of_et * 1000;
	pitch_de = PID_Cloud_Pitch.diff_of_et * 1000;
	
	pitch_s_et = PID_Cloud_Pitch_Speed.e_t * 1000;
	pitch_s_ut = PID_Cloud_Pitch_Speed.u_t *1000;
	pitch_s_Ie = PID_Cloud_Pitch_Speed.integ_of_et * 1000;
	pitch_s_de = PID_Cloud_Pitch_Speed.diff_of_et * 1000;

	CAN1_TxData16[0] = Motor_Yaw_Data.TargetCurrent;
	CAN1_TxData16[1] = Motor_Pitch_Data.TargetCurrent;
	
	can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);
}
/**
* @brief  云台陀螺仪PID
* @details  
* @param[in]  
* @retval  
*/
void Cloud_RunMPU(float pitch, float yaw)
{
	//陀螺仪为反馈的串级PID
	static float tmpPitch, tmpYaw , tmpPitchPow , tmpYawPow;
	Motor_PID_Loop_Section_rt( &PID_Cloud_Pitch , pitch , imu.soft.Pitch , 	&tmpPitch );
	Motor_PID_Loop_Section_rt( &PID_Cloud_Yaw ,		yaw , 	imu.soft.Yaw , 		&tmpYaw );
	Motor_PID_Loop_Section_rt( &PID_Cloud_Pitch_Speed , tmpPitch ,	imu.Angle_Rate[1] , &tmpPitchPow );
	Motor_PID_Loop_Section_rt( &PID_Cloud_Yaw_Speed , 	tmpYaw ,		imu.Angle_Rate[2] , &tmpYawPow );
	
	CAN1_TxData16[0] = (int16_t)tmpYawPow;
	CAN1_TxData16[1] = (int16_t)tmpPitchPow;

	can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);

}

void Cloud_RunEcdrMpuMix(uint16_t ecdr_pitch, uint16_t ecdr_yaw)
{
	
	static float tmpPitch , tmpPitchPow ;
	Motor_Yaw_Data.TargetPosition		= ecdr_yaw;			//加载编码器YAW值

	PID_MixedFeedback_rt( &PID_Cloud_Pitch , ecdr_pitch , Motor_Pitch_Data.RealPosition , imu.Angle_Rate[1] , &tmpPitch );
	Motor_PID_Loop_Section_rt( &PID_Cloud_Pitch_Speed , tmpPitch ,	imu.Angle_Rate[1] , &tmpPitchPow );
	Motor_PID_Loop_Section(&PID_Cloud_Yaw);
	Motor_PID_Loop_Section(&PID_Cloud_Yaw_Speed);
	
	pitch_et = PID_Cloud_Pitch.e_t * 1000;
	pitch_ut = PID_Cloud_Pitch.u_t *1000;
	pitch_Ie = PID_Cloud_Pitch.integ_of_et * 1000;
	pitch_de = PID_Cloud_Pitch.diff_of_et * 1000;
	
	pitch_s_et = PID_Cloud_Pitch_Speed.e_t * 1000;
	pitch_s_ut = PID_Cloud_Pitch_Speed.u_t *1000;
	pitch_s_Ie = PID_Cloud_Pitch_Speed.integ_of_et * 1000;
	pitch_s_de = PID_Cloud_Pitch_Speed.diff_of_et * 1000;

	CAN1_TxData16[0] = Motor_Yaw_Data.TargetCurrent;
	CAN1_TxData16[1] = (int16_t)tmpPitchPow;
	
	can_send_msg(&hcan1, 0x1FF, CAN1_TxData16);

}

