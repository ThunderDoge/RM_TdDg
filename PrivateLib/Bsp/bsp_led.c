/** 
* @brief    LED�弶֧�ְ�
* @details  ��ʼ��LED�������״̬
* @author   ������
* @date      2019.10
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*/
#include "bsp_led.h"

/**
* @brief  LED��״̬��ʼ��
* @details  ��ʼ��LED������Ϊ�͵�ƽ
* @param  NULL
* @retval  NULL
*/
void bsp_led_Init(void)
{
	HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,GPIO_PIN_RESET);
}
