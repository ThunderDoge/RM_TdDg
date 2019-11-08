#include "app_pid.h"
/** 
  * @brief      PID��ʼ��
  * @param[in]  pPID	ָ��PID_t�ṹ���ָ��
  * @param[in]  pMotor	ָ��Motor_t_c�ṹ���ָ��
  * @param[in]  Kp_t	��ʼ����Kp
  * @param[in]  Ki_t	��ʼ����Ki
  * @param[in]  Kd_t	��ʼ����Kd
  * @retval     
  * @pa         
  *             vx.x:
  */
	
//ȫ�ֱ����Ա�DEBUG

//PID_t PID_0;
//PID_t PID_Speed,PID_Angle;




/**
* @brief  
* @details  
* @param[in]  
* @retval  
*/

//void PID_Init(PID_InitTypeDef* PID_InitStructure)
//{
//	#define pPID PID_InitStructure->pPID
//	#define The PID_InitStructure->
//	pPID->pRtLoad = The pRt;
//	pPID->pMtLoad = The pMt;
//	pPID->pUtSave = The pUt;
//	
//	pPID->K_p 				= The Kp_t;
//	pPID->K_i 				= The Ki_t;
//	pPID->K_d 				= The Kd_t;
//	
//	pPID->eIntegralMax=	The eIntegralAbsMax;
//	pPID->eIntegralMin= The eIntegralMin;
//	pPID->OutputMax		= The OutputMax;
//	pPID->OutputMin		= The OutputMin;
//	
//	pPID->u_t					= 0;
//	pPID->e_t 				= 0;
//	pPID->prev_e_t 		= 0;
//	pPID->integ_of_et = 0;
//	pPID->diff_of_et 	= 0;
//	#undef pPID
//	#undef The
//}

/**
* @brief  �����˲���ʼ��
* @details  
* @param[in]  
* @retval  
*/

void SlidingFilter_Init(SlidingFilter* pFilter, uint16_t window_lenght)
{
	pFilter->buffer_iter_start = 0;
	pFilter->buffer_iter_end = window_lenght;
	memset((void*)pFilter->slide_buffer , 0 , sizeof(int16_t)*SLIDING_BUFFER_LENGTH);
	pFilter->sum_of_buffer = 0;
}
/**
* @brief  �����˲����к���������ʹ�û�������
* @details  
* @param[in]  
* @retval  ���������ݺ�����ֵ
*/

float SlidingFilter_Run(SlidingFilter* pFilter,int16_t newInput)
{
	pFilter->buffer_iter_end ++;
	if( pFilter->buffer_iter_end >=  SLIDING_BUFFER_LENGTH)	//���������ͷ
	{
		pFilter->buffer_iter_end = 0;
	}
	
	pFilter->slide_buffer [ pFilter->buffer_iter_end ] = newInput;	 //����������
	pFilter->sum_of_buffer += newInput;															//����֮���������µ�����
	
	pFilter->buffer_iter_start ++;
	if( pFilter->buffer_iter_start >=  SLIDING_BUFFER_LENGTH) //���������ͷ
	{
		pFilter->buffer_iter_start = 0;
	}
	pFilter->sum_of_buffer -= pFilter->slide_buffer[ pFilter->buffer_iter_start ];	//����֮�ͳ�ȥ�������
	pFilter->output = (pFilter->sum_of_buffer) / (float)pFilter->window_lenght;			//�ó����
	return pFilter->output;													//����ֵ ���
}
/**
* @brief  PID��ʼ������
* @details  
* @param[in]  
* @retval  
*/

void PID_Init( PID_t* pPID , int16_t* pRt , int16_t* pMt , int16_t* pUt ,
float Kp_t , float Ki_t , float Kd_t, int16_t eIntegralMax , int16_t OutputMax)
{
	pPID->pRtLoad =  pRt;
	pPID->pMtLoad =  pMt;
	pPID->pUtSave =  pUt;
	
	pPID->K_p 				=  Kp_t;
	pPID->K_i 				=  Ki_t;
	pPID->K_d 				=  Kd_t;
	
	pPID->eIntegralAbsMax	=	 eIntegralMax;
	pPID->OutputAbsMax		=  OutputMax;
	if(pPID->IntegralNeighbourAbs == 0)
	{	pPID->IntegralNeighbourAbs = 3e+38F; }
	else
		pPID->IntegralNeighbourAbs = fabs(pPID->IntegralNeighbourAbs);
	
	pPID->u_t					= 0;
	pPID->e_t 				= 0;
	pPID->prev_e_t 		= 0;
	pPID->integ_of_et = 0;
	pPID->diff_of_et 	= 0;
	
	#ifdef USE_SLIDING_FILTER
	SlidingFilter_Init( &pPID->sf, 40);
	#endif
}
void PID_Init_rt(PID_t* pPID , 
								float Kp_t , float Ki_t , float Kd_t, int16_t eIntegralMax , int16_t OutputMax)
{
	pPID->K_p 				=  Kp_t;
	pPID->K_i 				=  Ki_t;
	pPID->K_d 				=  Kd_t;
	
	pPID->eIntegralAbsMax	=	 eIntegralMax;
	pPID->OutputAbsMax		=  OutputMax;
	
	pPID->u_t					= 0;
	pPID->e_t 				= 0;
	pPID->prev_e_t 		= 0;
	pPID->integ_of_et = 0;
	pPID->diff_of_et 	= 0;
	
	#ifdef USE_SLIDING_FILTER
	SlidingFilter_Init( &pPID->sf, 40);
	#endif

}


/**
* @brief  �� app_quad_math �����õı����޷�������Ϊ�˱����ظ�����ʹ����static
* @details  
* @param[in]	data ���޷��ı�����ֵ
* @retval  �޷��������ֵ
*/

static float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}


/** 
  * @brief      PID���к�����������ѭ����
  * @param[in]  pPID	ָ���õ���PID_t�ṹ���ָ��;
  * @retval     
  * @pa         
  *             v0.1:
  */


void Motor_PID_Loop_Section(PID_t* pPID)
{
	pPID->prev_e_t = pPID->e_t;
	pPID->e_t = *(pPID->pRtLoad) - *(pPID->pMtLoad);
	pPID->integ_of_et += pPID->e_t;
	#ifdef USE_SLIDING_FILTER
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//ʹ�û����˲���
	#endif
	pPID->diff_of_et = pPID->e_t - pPID->prev_e_t;
	
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));
	
	
	pPID->u_t = (pPID->K_p*pPID->e_t 
							+ pPID->K_i*pPID->integ_of_et 
							+ pPID->K_d* pPID->diff_of_et);
	
	pPID->u_t = Limit(pPID->u_t, pPID->OutputAbsMax, -(pPID->OutputAbsMax));
	
	*(pPID->pUtSave) = pPID->u_t;
}
/**
* @brief  ��Ӧ�ó��������������趨ֵ����������������к�����������ĵ�Ч���������ʹ�á�
* @details  
* @param[in]  
* @retval  
*/

void Motor_PID_Loop_Section_rt( PID_t* pPID, float Target, float Measure, float* Output)
{
	pPID->prev_e_t = pPID->e_t;
	pPID->e_t = Target - Measure ;
	pPID->integ_of_et += pPID->e_t;
	#ifdef USE_SLIDING_FILTER
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//ʹ�û����˲���
	#endif
	pPID->diff_of_et = pPID->e_t - pPID->prev_e_t;
	
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));
	
	
	pPID->u_t = (pPID->K_p*pPID->e_t 
							+ pPID->K_i*pPID->integ_of_et 
							+ pPID->K_d* pPID->diff_of_et);
	
	pPID->u_t = Limit(pPID->u_t, pPID->OutputAbsMax, -(pPID->OutputAbsMax));
	
	*Output = pPID->u_t;
}
/**
* @brief  ���ַ���PID
* @details  
* @param[in]  
* @retval  
*/

void PID_LoopSectionInteSepara_rt( PID_t* pPID, float Target, float Measure, float* Output)
{
	pPID->prev_e_t = pPID->e_t;
	pPID->e_t = Target - Measure ;
	pPID->integ_of_et += pPID->e_t;
	#ifdef USE_SLIDING_FILTER
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//ʹ�û����˲���
	#endif
	pPID->diff_of_et = pPID->e_t - pPID->prev_e_t;
	
	if( fabs(pPID->e_t) > pPID->IntegralNeighbourAbs)
		{ pPID->integ_of_et = 0; }												//�����������䣬��������
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));		//�����޷�
	
	
	pPID->u_t = (pPID->K_p*pPID->e_t 
							+ pPID->K_i*pPID->integ_of_et 
							+ pPID->K_d* pPID->diff_of_et);
	
	pPID->u_t = Limit(pPID->u_t, pPID->OutputAbsMax, -(pPID->OutputAbsMax));
	
	*Output = pPID->u_t;

}

void PID_MixedFeedback_rt( PID_t* pPID, float Target, float Measure, float RateOfErr, float* Output )
{
	pPID->prev_e_t = pPID->e_t;
	pPID->e_t = Target - Measure ;
	pPID->integ_of_et += pPID->e_t;
	#ifdef USE_SLIDING_FILTER
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//ʹ�û����˲���
	#endif
	pPID->diff_of_et = RateOfErr;
	
	if( fabs(pPID->e_t) > pPID->IntegralNeighbourAbs)
		{ pPID->integ_of_et = 0; }												//�����������䣬��������
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));		//�����޷�
	
	
	pPID->u_t = (pPID->K_p*pPID->e_t 
							+ pPID->K_i*pPID->integ_of_et 
							+ pPID->K_d* pPID->diff_of_et);
	
	pPID->u_t = Limit(pPID->u_t, pPID->OutputAbsMax, -(pPID->OutputAbsMax));
	
	*Output = pPID->u_t;

}




