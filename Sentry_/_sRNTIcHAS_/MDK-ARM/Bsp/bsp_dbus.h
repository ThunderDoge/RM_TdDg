#ifndef __BSP_DBUS_H
#define __BSP_DBUS_H
#include "stm32f4xx.h"
#include "string.h"
#include "usart.h"

//�������뿪�غ궨�壬����Ҫ�����ľ�ע�͵���غ궨��
//#define BSP_DBUS_USE_SIGNAL
//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_DBUS_UART						huart2
//Dbus���ջ���������С������Ҫ���������޸�,ʹ�ÿ����ж�Ҫ�����������Ҫ����18
#define BSP_DBUS_BUFFER_SIZE		25

//ң����һЩ��صĺ궨��,�ӹٷ��ֲῴ�����ģ��������Ҫ��ע���˰�
#define BSP_DBUS_RC_CH_MIN			((uint16_t)364)
#define BSP_DBUS_RC_CH_MID			((uint16_t)1024)
#define BSP_DBUS_RC_CH_MAX			((uint16_t)1684)
#define BSP_DBUS_RC_SW_UP				((uint8_t)1)
#define BSP_DBUS_RC_SW_MID			((uint8_t)3)
#define BSP_DBUS_RC_sW_DOWN			((uint8_t)2)
//������һ���Ͻ���룬�˴�����������ܺ͹ٷ��ֲ��в��죬��������Լ�����һ��
#define BSP_DBUS_KEY_W					((uint16_t)0x01<<0) 
#define BSP_DBUS_KEY_S					((uint16_t)0x01<<1)
#define BSP_DBUS_KEY_A					((uint16_t)0x01<<2)
#define BSP_DBUS_KEY_D					((uint16_t)0x01<<3)
#define BSP_DBUS_KEY_Q					((uint16_t)0x01<<7)
#define BSP_DBUS_KEY_E					((uint16_t)0x01<<8)
#define BSP_DBUS_KEY_CTRL				((uint16_t)0x01<<6)
#define BSP_DBUS_KEY_SHIFT			((uint16_t)0x01<<5)
#define BSP_DBUS_KEY_R					((uint16_t)0x01<<9) 
#define BSP_DBUS_KEY_F					((uint16_t)0x01<<10)
#define BSP_DBUS_KEY_G					((uint16_t)0x01<<11)
#define BSP_DBUS_KEY_Z					((uint16_t)0x01<<12)
#define BSP_DBUS_KEY_X					((uint16_t)0x01<<13)
#define BSP_DBUS_KEY_C					((uint16_t)0x01<<14) 
#define BSP_DBUS_KEY_V					((uint16_t)0x01<<15)
#define BSP_DBUS_KEY_B					((uint16_t)0x01<<16)

/*ң�����ݽṹ��*/
typedef struct rc_rec
{
		int16_t CH_0;		//ͨ��0
		int16_t CH_1;		//ͨ��1
		int16_t CH_2;		//ͨ��2
		int16_t CH_3;		//ͨ��3
		uint8_t S1;			//���뿪��
		uint8_t S2;			//�Ҳ��뿪��
		int16_t Dial;		//������
		struct 
		{
			int16_t X;				//X��
			int16_t Y;				//Y��
			int16_t Z;				//Z��
			uint8_t Leftkey;	//�Ҽ�
			uint8_t Rightkey;	//�Ҽ�
		}Mouse;					//�����Ϣ
		uint16_t Keys;	//������Ϣ
}bsp_dbus_RC_Data;

extern bsp_dbus_RC_Data bsp_dbus_Data;

//�����������������ʹ���ź�����������ṩһ������������Ϊ�ӿ�
#ifdef	BSP_DBUS_USE_SIGNAL
	void bsp_dbus_It(void);							//Dbus�жϴ�����
	void bsp_dbus_Init(void);						//Dbus��ʼ������
	void bsp_dbus_Analysis(void);				//Dbus���ݽ���������ʹ���ź���ʱ����⿪�Žӿ�
#else
	void bsp_dbus_It(void);							//Dbus�жϴ�����.���ж��е���
	void bsp_dbus_Init(void);						//Dbus��ʼ������
#endif

#endif
