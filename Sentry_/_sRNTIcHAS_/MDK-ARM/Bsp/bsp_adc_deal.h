#ifndef  __BSP_ADC_DEAL_H
#define  __BSP_ADC_DEAL_H
#include "adc.h"
#define bsp_ADC_USE_ADC1 hadc1
//#define bsp_ADC_USE_ADC2 hadc2

extern uint16_t bsp_Final_ADC1_Volage[4];  // ���������ĵ�ѹ���
extern uint16_t bsp_ADC1_Value[400];  //�洢���вɼ��������� 
extern float    bsp_ADC1_Sharp_Distance[4];  // ���պ����࣬������ϵ�

extern uint16_t bsp_Final_ADC2_Volage[4];// ���������ĵ�ѹ���
extern uint16_t bsp_ADC2_Value[400];  //�洢���вɼ���������
extern float    bsp_ADC2_Sharp_Distance[4];  // ���պ����࣬������ϵ�
void bsp_ADC_Sensor_Init();  //��ʼ��ADC



#endif  /*__ADC_DEAL_H*/
