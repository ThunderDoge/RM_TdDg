#include "app_pid.h"
/** 
  * @brief      PID初始化
  * @param[in]  pPID	指向PID_t结构体的指针
  * @param[in]  pMotor	指向Motor_t_c结构体的指针
  * @param[in]  Kp_t	初始化用Kp
  * @param[in]  Ki_t	初始化用Ki
  * @param[in]  Kd_t	初始化用Kd
  * @retval     
  * @pa         
  *             vx.x:
  */
	
//全局变量以便DEBUG

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
* @brief  滑动滤波初始化
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
* @brief  滑动滤波运行函数，缓存使用环形数组
* @details  
* @param[in]  
* @retval  加入新数据后的输出值
*/

float SlidingFilter_Run(SlidingFilter* pFilter,int16_t newInput)
{
	pFilter->buffer_iter_end ++;
	if( pFilter->buffer_iter_end >=  SLIDING_BUFFER_LENGTH)	//环形数组回头
	{
		pFilter->buffer_iter_end = 0;
	}
	
	pFilter->slide_buffer [ pFilter->buffer_iter_end ] = newInput;	 //插入新数据
	pFilter->sum_of_buffer += newInput;															//数据之和增加最新的数据
	
	pFilter->buffer_iter_start ++;
	if( pFilter->buffer_iter_start >=  SLIDING_BUFFER_LENGTH) //环形数组回头
	{
		pFilter->buffer_iter_start = 0;
	}
	pFilter->sum_of_buffer -= pFilter->slide_buffer[ pFilter->buffer_iter_start ];	//数据之和除去最旧数据
	pFilter->output = (pFilter->sum_of_buffer) / (float)pFilter->window_lenght;			//得出输出
	return pFilter->output;													//返回值 输出
}
/**
* @brief  PID初始化函数
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
* @brief  从 app_quad_math 处借用的变量限幅函数。为了避免重复定义使用了static
* @details  
* @param[in]	data 待限幅的变量的值
* @retval  限幅后变量的值
*/

static float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}


/** 
  * @brief      PID运行函数，放置在循环中
  * @param[in]  pPID	指向用到的PID_t结构体的指针;
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
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//使用滑动滤波器
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
* @brief  在应用程序中由语句决定设定值、反馈和输出的运行函数。与上面的等效，根据情况使用。
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
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//使用滑动滤波器
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
* @brief  积分分离PID
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
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//使用滑动滤波器
	#endif
	pPID->diff_of_et = pPID->e_t - pPID->prev_e_t;
	
	if( fabs(pPID->e_t) > pPID->IntegralNeighbourAbs)
		{ pPID->integ_of_et = 0; }												//超出积分区间，积分清零
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));		//积分限幅
	
	
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
	//SlidingFilter_Run(&pPID->sf , pPID->e_t);			//使用滑动滤波器
	#endif
	pPID->diff_of_et = RateOfErr;
	
	if( fabs(pPID->e_t) > pPID->IntegralNeighbourAbs)
		{ pPID->integ_of_et = 0; }												//超出积分区间，积分清零
	pPID->integ_of_et = Limit(pPID->integ_of_et,pPID->eIntegralAbsMax, -(pPID->eIntegralAbsMax));		//积分限幅
	
	
	pPID->u_t = (pPID->K_p*pPID->e_t 
							+ pPID->K_i*pPID->integ_of_et 
							+ pPID->K_d* pPID->diff_of_et);
	
	pPID->u_t = Limit(pPID->u_t, pPID->OutputAbsMax, -(pPID->OutputAbsMax));
	
	*Output = pPID->u_t;

}




