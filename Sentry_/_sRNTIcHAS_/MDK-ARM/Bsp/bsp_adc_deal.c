/** 
* @brief    ���պ�������Ҫ��4��ͨ���ĵ������ݽ��вɼ�����
* @details  ADC�жϻص������������ݽ��ա��˲���Ҫ����ٸ������ݴ���
* @author   TAOU Monkey
* @date      2019.11
* @version  1.0
* @par Copyright (c):  RM2020���
* @par 
* @par     ����ļ�������ΪADC1_DMA�ж������������õ���bsp_ADC_Value[400]
* @par     �����Ľ��Ϊbsp_Final_ADC_Volage[4]
* @par     ������պ����������Ϊbsp_ADC1_Sharp_Distance��bsp_ADC2_Sharp_Distance
*				ʹ�÷�����1.��ͷ�ļ��ж����Լ���Ҫ��ADC1/2
                  2.����ʼ������bsp_ADC_Sensor_Init()��ʼ������
*				�汾���:1.0�����19��ԭʼ�汾���Ը��ӷ���ؿ����Լ����õ�ADCͨ��
*       2019.11.27  9��35         1.1���մ���淶���±�д

*/
 
#include "bsp_adc_deal.h"
#include "math.h"

#ifdef bsp_ADC_USE_ADC1
uint16_t bsp_ADC1_Value[400];   //�洢���вɼ��������� 
//���������ͨ���ɼ���������
uint16_t ADC1_ch1_value[100];
uint16_t ADC1_ch2_value[100];
uint16_t ADC1_ch3_value[100];
uint16_t ADC1_ch4_value[100];
float ADC1_fixed_ch[4];     //��ž�ֵ�˲����4����ѹ����  [4][1]
int   ADC1_adjust_offset=0; //��ʼ��ѹ��ȡ����������
uint16_t ADC1_ch_offset[4] = {0};  //�ų�ʼ��ѹ
float 	   ADC1_que[4][10];  //���10�εľ�ֵ�˲����
float   ADC1_ch_sum[4]={0};  //���ÿһ��ͨ��10�ξ�ֵ�˲������ƽ��ֵ
uint16_t bsp_Final_ADC1_Volage[4];  // ���������ĵ�ѹ���
float    bsp_ADC1_Sharp_Distance[4];    // ���պ����࣬������ϵ�

#endif

#ifdef bsp_ADC_USE_ADC2
uint16_t bsp_ADC2_Value[400];   //�洢���вɼ��������� 

uint16_t ADC2_ch1_value[100];
uint16_t ADC2_ch2_value[100];
uint16_t ADC2_ch3_value[100];
uint16_t ADC2_ch4_value[100];
float ADC2_fixed_ch[4];     //��ž�ֵ�˲����4����ѹ����  [4][1]
int   ADC2_adjust_offset=0; //��ʼ��ѹ��ȡ����������
uint16_t ADC2_ch_offset[4] = {0};  //�ų�ʼ��ѹ
float 	   ADC2_que[4][10];  //���10�εľ�ֵ�˲����
float   ADC2_ch_sum[4]={0};  //���ÿһ��ͨ��10�ξ�ֵ�˲������ƽ��ֵ
uint16_t bsp_Final_ADC2_Volage[4];  // ���������ĵ�ѹ���
float    bsp_ADC2_Sharp_Distance[4];    // ���պ����࣬������ϵ�
#endif

/*******************************************************************************************
  * @Func			get_4ch(unsigned short int *signal,unsigned short int *channel,int length,unsigned char mask)
  * @Brief    ����mask��ֵ��ȡ�ĸ�ͨ���Ĳ���ֵ(��ADC�Ļص������д�������ʹ��)
  * @Param		signalΪ�ɼ��������ݣ�channelΪ������ĵ���ͨ�������ݣ�lengthΪ����ͨ�������ݳ���
  * @Retval		None
* @Date     2015/11/24    2018��10��30��16:49:07  Nankel�޸�
 *******************************************************************************************/
void get_4ch(unsigned short int *signal,unsigned short int *channel,int length,unsigned char mask)
{
	for(int j=0;j<length/4;j++)
		channel[j] = signal[j*4 + mask];
}

/**********ȥ�����ֵ����Сֵ����ƽ��ֵ************/
float mid_sum_filter(uint16_t *data,uint16_t num)
{
	uint8_t i=0;
	uint16_t max=0,min=0;
	uint32_t sum=0;
	float average=0;
	
	max = *(data+0);
	min = *(data+0);
	sum += *(data+0);
	
	for (i=1; i<num; i++){
		if (max < *(data+i))
			max = *(data+i);
 		if (min > *(data+i))
			min = *(data+i);
		sum += *(data+i);
	}
	sum = sum - max - min;
	average = (float)sum/(num-2);
	return 	average;
}

/** 
  * @brief      ADC�жϻص��������������ݽ��պ��˲�
  * @param[in]  ADC_HandleTypeDef* hadc
  * @retval      
  * @retval       
  * @pa      
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	
#ifdef bsp_ADC_USE_ADC1
	if(hadc == &hadc1){
		get_4ch(bsp_ADC1_Value,ADC1_ch1_value,400,0);
		get_4ch(bsp_ADC1_Value,ADC1_ch2_value,400,1);
		get_4ch(bsp_ADC1_Value,ADC1_ch3_value,400,2);
		get_4ch(bsp_ADC1_Value,ADC1_ch4_value,400,3);
		ADC1_fixed_ch[0] = mid_sum_filter(ADC1_ch1_value, 100);         
		ADC1_fixed_ch[1] = mid_sum_filter(ADC1_ch2_value, 100);          
		ADC1_fixed_ch[2] = mid_sum_filter(ADC1_ch3_value, 100);
		ADC1_fixed_ch[3] = mid_sum_filter(ADC1_ch4_value, 100);
		

		for(int i=0; i<4; i++){
			for(int k=9; k>0; k--)   //��[4][10]�����������λһ�� 
				ADC1_que[i][k] = ADC1_que[i][k-1];
			ADC1_que[i][0] = ADC1_fixed_ch[i];	//��ADC��������ƽ��ֵ�����һ��				
			for(int m=0; m<10; m++)   //��ÿ�е�Ԫ�ؼ�����	
				ADC1_ch_sum[i] += ADC1_que[i][m];					
			ADC1_ch_sum[i] /= 10.0f;	
			bsp_Final_ADC1_Volage[i] =  (uint16_t) (ADC1_ch_sum[i] - ADC1_ch_offset[i]);    
		}
		for(uint8_t i=0;i<2;i++){
			if(bsp_Final_ADC1_Volage[i] < 750)  bsp_Final_ADC1_Volage[i] = 750;  // ��Ӧ60cm����
			bsp_ADC1_Sharp_Distance[i] = 284486*pow(bsp_Final_ADC1_Volage[i],-1.275);
		}

  }
#endif
	
#ifdef bsp_ADC_USE_ADC2
	if(hadc == &hadc2){
		get_4ch(bsp_ADC2_Value,ADC2_ch1_value,400,0);
		get_4ch(bsp_ADC2_Value,ADC2_ch2_value,400,1);
		get_4ch(bsp_ADC2_Value,ADC2_ch3_value,400,2);
		get_4ch(bsp_ADC2_Value,ADC2_ch4_value,400,3);
		ADC2_fixed_ch[0] = mid_sum_filter(ADC2_ch1_value, 100);         
		ADC2_fixed_ch[1] = mid_sum_filter(ADC2_ch2_value, 100);          
		ADC2_fixed_ch[2] = mid_sum_filter(ADC2_ch3_value, 100);
		ADC2_fixed_ch[3] = mid_sum_filter(ADC2_ch4_value, 100);
		

		for(int i=0; i<4; i++){
			for(int k=9; k>0; k--)   //��[4][10]�����������λһ�� 
				ADC2_que[i][k] = ADC2_que[i][k-1];
			ADC2_que[i][0] = ADC2_fixed_ch[i];	//��ADC��������ƽ��ֵ�����һ��				
			for(int m=0; m<10; m++)   //��ÿ�е�Ԫ�ؼ�����	
				ADC2_ch_sum[i] += ADC2_que[i][m];					
			ADC2_ch_sum[i] /= 10.0f;	
			bsp_Final_ADC2_Volage[i] =  (uint16_t) (ADC2_ch_sum[i] - ADC2_ch_offset[i]);    
		}

	}
#endif 
}




/** 
  * @brief      ��������ʼ������
  * @param[in]  ��ĳ���ط���ʼ������
  * @retval      
  * @retval       
  * @pa      
  */

void bsp_ADC_Sensor_Init()
{
	#ifdef bsp_ADC_USE_ADC1
 	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)bsp_ADC1_Value,400);//���⼤�������ݲɼ�
	#endif
	#ifdef bsp_ADC_USE_ADC2
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)bsp_ADC2_Value,400);//���⼤�������ݲɼ�
	#endif
}
