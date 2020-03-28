/**
 * @file bsp_dt.cpp
 * @brief ���ݴ���弶֧�ְ�
 * @author Evan-GH (751191269@qq.com)
 * @version 1.4
 * @date 2020-02-28
 * @copyright OnePointFive
 * @par ��־:
 *   V1.0 �������ļ�������ⷢ�����ݵ���غ���������\n
 *   V1.1 ���¹淶�������ļ������ʽ��Doxygenע��\n
 *   V1.2 ���Ʒ��ͺͽ��մ������غ���\n
 *   V1.3 ������Ӷ�������λ�������ݴ���֧��\n
 *   v1.4 ��ɶ���������λ��V7�汾��Э�������\n
 */
#include "bsp_dt.hpp"
#include "usart.h"
#include "app_imu.h"
#include <string.h>
#include "bsp_dbus.h"

//ʹ��������λ������������Ҫ�ĺ궨�壬���ڲ��һ���������ĸߵ�λ
#define BYTE0(Data)      (*(char *)(&Data))
#define BYTE1(Data)      (*((char *)(&Data)+1))
#define BYTE2(Data)      (*((char *)(&Data)+2))
#define BYTE3(Data)      (*((char *)(&Data)+3))

bsp_dt_data bsp_dt_Send_Data; //!< �洢���ⷢ����Ϣ�Ľṹ��
static uint8_t DT_Rxbuffer[BSP_DT_BUFFER_SIZE]={0}; //!< ���ڽ������ݻ�������
int32_t bsp_dt_ParList[166];  //�����б�

static uint8_t DT_Txbuffer[16]={0};  //���ڷ���������
/**
 * @brief ʹ���Լ�����λ���������ݵĴ��ڷ��ͺ���
 * @return HAL_StatusTypeDef
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_Sendmessage(void)
{
	int16_t _check_sum = 0; //��У���ñ���
	memset(DT_Txbuffer,0,16); //����֮ǰ�����һ��
	DT_Txbuffer[0] = 0xff; //֡ͷ
	DT_Txbuffer[1] = bsp_dt_Send_Data.Function_word;  //������
	DT_Txbuffer[15] = 0x0d; //֡β

	memcpy(DT_Txbuffer+2,&bsp_dt_Send_Data.Data_1,4);
	memcpy(DT_Txbuffer+6,&bsp_dt_Send_Data.Data_2,4);
	memcpy(DT_Txbuffer+10,&bsp_dt_Send_Data.Data_3,4);

	for(int i=0; i<16; i++) //��У��
	{
		if(i!=14)	_check_sum+=DT_Txbuffer[i];  //���˺�У����һλ����λ�ϵ���ֵ�Ӻ�
	}
	_check_sum = _check_sum & 0xff;
	DT_Txbuffer[14] = _check_sum;

	return HAL_UART_Transmit_DMA(&BSP_DT_UART,DT_Txbuffer,16);
}

static uint8_t ANO_DT_Txbuffer[50] = {0};  //���ڷ����û��棬������λ����
/**
 * @brief ��������λ�������û��Զ�������
 * @param _Frame_ID ֡ID
 * @param Data1 ����1
 * @param Data2 ����2
 * @param Data3 ����3
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_UserData(uint8_t _Frame_ID, int32_t Data1, int32_t Data2, int32_t Data3)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = _Frame_ID;  //�����û��Զ�������
	ANO_DT_Txbuffer[cnt++] = 12;  //���ݳ���

	int32_t temp = Data1;  //���Data1
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);
	temp = Data2;  //���Data2
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);
	temp = Data3;  //���Data3
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ�����ʹ���������1
 * @param Acc_x X����ٶ�
 * @param Acc_y Y����ٶ�
 * @param Acc_z Z����ٶ�
 * @param Gyro_x X����ٶ�
 * @param Gyro_y Y����ٶ�
 * @param Gyro_z Z����ٶ�
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData1(int16_t Acc_x, int16_t Acc_y, int16_t Acc_z, int16_t Gyro_x, int16_t Gyro_y, int16_t Gyro_z)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0X01;  //���ʹ���������1
	ANO_DT_Txbuffer[cnt++] = 13;  //���ݳ���

	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_x);  //���X����ٶ�
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_y);  //���Y����ٶ�
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Acc_z);  //���Z����ٶ�
	ANO_DT_Txbuffer[cnt++] = BYTE1(Acc_z);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_x);  //���X������������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_y);  //���Y������������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Gyro_z);  //���Z������������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Gyro_z);
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ�����ʹ���������2���¶�����ʮ��
 * @param Mag_x X�����������
 * @param Mag_y Y�����������
 * @param Mag_z Z�����������
 * @param Alt_Bar ��ѹ�Ƹ߶����ݣ���λCM
 * @param Temp �¶�����
 * @param Bar_STA ��ѹ��״̬
 * @param Mag_STA ������״̬
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_SensorData2(int16_t Mag_x, int16_t Mag_y, int16_t Mag_z, float Alt_Bar, float Temp, uint8_t Bar_STA, uint8_t Mag_STA)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0X02;  //���ʹ���������2
	ANO_DT_Txbuffer[cnt++] = 14;  //���ݳ���

	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_x);  //���X�����������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_x);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_y);  //���Y�����������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_y);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_z);  //���Z�����������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Mag_z);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Alt_Bar);  //�����ѹ�Ƹ߶�����
	ANO_DT_Txbuffer[cnt++] = BYTE1(Alt_Bar);
	ANO_DT_Txbuffer[cnt++] = BYTE2(Alt_Bar);
	ANO_DT_Txbuffer[cnt++] = BYTE3(Alt_Bar);
	int16_t temp = Temp * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //����¶�����
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Bar_STA);  //���״̬����
	ANO_DT_Txbuffer[cnt++] = BYTE0(Mag_STA);

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ������ŷ�������ݣ���������100��
 * @param Pitch ������
 * @param Roll ������
 * @param Yaw ƫ����
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_EulerAngle(float Pitch, float Roll, float Yaw)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xAF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0X03;  //����ŷ����
	ANO_DT_Txbuffer[cnt++] = 0x07;  //���ݳ���

	int16_t temp;
	//��һ���Ƕ�
	if(Roll>0 && Roll<180)
		temp = (180-Roll)*100;
	if(Roll<0 && Roll>-180)
		temp = (-180-Roll)*100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //��䷭����
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Pitch * 100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //��丩����
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Yaw * 100;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //���ƫ����
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ������ң��������
 * @param CH0 ͨ��0����
 * @param CH1 ͨ��1����
 * @param CH2 ͨ��2����
 * @param CH3 ͨ��3����
 * @param S1 ���뿪��1
 * @param S2 ���뿪��2
 * @param Dial ��������
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Remote(int16_t CH0, int16_t CH1, int16_t CH2, int16_t CH3, int16_t S1, int16_t S2, int16_t Dial)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0X40;  //����ң��������
	ANO_DT_Txbuffer[cnt++] = 20;  //���ݳ���

	ANO_DT_Txbuffer[cnt++] = BYTE0(CH0);  //���ͨ��0����
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH0);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH1);  //���ͨ��1����
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH1);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH2);  //���ͨ��2����
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH2);
	ANO_DT_Txbuffer[cnt++] = BYTE0(CH3);  //���ͨ��3����
	ANO_DT_Txbuffer[cnt++] = BYTE1(CH3);
	ANO_DT_Txbuffer[cnt++] = BYTE0(S1);  //��䲦�뿪��1����
	ANO_DT_Txbuffer[cnt++] = BYTE1(S1);
	ANO_DT_Txbuffer[cnt++] = BYTE0(S2);  //��䲦�뿪��2����
	ANO_DT_Txbuffer[cnt++] = BYTE1(S2);
	ANO_DT_Txbuffer[cnt++] = BYTE0(Dial);  //��䲦������
	ANO_DT_Txbuffer[cnt++] = BYTE1(Dial);

	//���������û�оͲ�0
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = 0;

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ�����͵�ѹ�������ݣ���������ʮ��
 * @param Volatile ��ѹ����
 * @param Current ��������
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_VolatileCurrent(float Volatile, float Current)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0X0D;  //����ң��������
	ANO_DT_Txbuffer[cnt++] = 4;  //���ݳ���

	uint16_t temp = Volatile * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //����ѹ����
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	temp = Current * 10;
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);  //����������
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ������һ���ַ���
 * @param Color �ַ�����ɫ
 * @param String ���͵��ַ���
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_LogString(uint8_t Color, uint8_t* String)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0XA0;  //���ʹ���������
	ANO_DT_Txbuffer[cnt++] = 0;
	ANO_DT_Txbuffer[cnt++] = Color;  //������ɫ
	uint8_t i = 0;
	while(*(String+i) != '\0')
	{
		ANO_DT_Txbuffer[cnt++] = *(String+i++);  //����ַ�����֡��
		if(cnt > 50) break;  //֡�������50
	}
	ANO_DT_Txbuffer[3] = cnt - 4;
	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ������log��Ϣ�����֣��ַ���
 * @param Num ��ֵ
 * @param String �ַ�����Ϣ
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_NumString(int32_t Num,uint8_t *String)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0XA0;  //���ʹ���������
	ANO_DT_Txbuffer[cnt++] = 0;

	ANO_DT_Txbuffer[cnt++] = BYTE0(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE1(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE2(Num);
	ANO_DT_Txbuffer[cnt++] = BYTE3(Num);

	uint8_t i = 0;
	while(*(String+i) != '\0')
	{
		ANO_DT_Txbuffer[cnt++] = *(String+i++);  //����ַ�����֡��
		if(cnt > 50) break;  //֡�������50
	}
	ANO_DT_Txbuffer[3] = cnt - 4;
	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ����У����Ϣ����λ��
 * @param _Sc �յ��ĺ�У��
 * @param _Ac �յ��ĸ���У��
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Check(uint8_t Frame_ID, uint8_t _Sc, uint8_t _Ac)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0x00;  //���ͷ���֡
	ANO_DT_Txbuffer[cnt++] = 3;  //���ݳ���
	ANO_DT_Txbuffer[cnt++] = Frame_ID;
	ANO_DT_Txbuffer[cnt++] = _Sc;  //�����յ���У����Ϣ
	ANO_DT_Txbuffer[cnt++] = _Ac;

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

/**
 * @brief ��������λ�����ͱ���ѯ�Ĳ���ֵ��PID��������1000���ϴ�
 * @param _ParaID
 * @return HAL_StatusTypeDef ���ͽ��
 * @retval HAL_OK ���ͳɹ�
 * @retval HAL_ERROR ����ʧ��
 */
HAL_StatusTypeDef bsp_dt_ANO_Send_Parameter(uint16_t _ParaID)
{
	uint8_t cnt = 0;
	ANO_DT_Txbuffer[cnt++] = 0xAA;  //֡ͷ
	ANO_DT_Txbuffer[cnt++] = 0xFF;  //Ŀ���ַ
	ANO_DT_Txbuffer[cnt++] = 0xE2;  //���ͷ���֡
	ANO_DT_Txbuffer[cnt++] = 6;  //���ݳ���

	ANO_DT_Txbuffer[cnt++] = BYTE0(_ParaID);
	ANO_DT_Txbuffer[cnt++] = BYTE1(_ParaID);

	int32_t temp = bsp_dt_ParList[_ParaID];
	if(temp >= 11 && temp <=64)  //PID�����б���ʱ��Ҫ�Ѷ�Ӧ������������1000��
	{

	}
	ANO_DT_Txbuffer[cnt++] = BYTE0(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE1(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE2(temp);
	ANO_DT_Txbuffer[cnt++] = BYTE3(temp);

	uint8_t sc=0,ac=0;  //У��
	for(int i=0;i<ANO_DT_Txbuffer[3]+4;i++)
	{
		sc += ANO_DT_Txbuffer[i];
		ac += sc;
	}
	ANO_DT_Txbuffer[cnt++] = sc;
	ANO_DT_Txbuffer[cnt++] = ac;

	return HAL_UART_Transmit(&BSP_DT_UART,ANO_DT_Txbuffer,cnt,0xff);
}

static void bsp_dt_Ano_Data_Analysis(void)
{
	uint8_t sc=0,ac=0;  //У���ñ���
	uint16_t ParID;

	if (DT_Rxbuffer[0] == 0xAA && (DT_Rxbuffer[1] == 0xFF || DT_Rxbuffer[1] == 0x05))  //���֡��ͷ��������
	{
		switch (DT_Rxbuffer[2])
		{
			case 0xE0:  //�������
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //У�鲻ͨ��������
					return;
				bsp_dt_ANO_Send_Check(0XE0,sc,ac);  //���ͻ���ָ��
				if(DT_Rxbuffer[4] == 0x02)  //ϵͳ�������Ҫ�ڷ��ͷ�������֮��ִ�У�����ᷴ������
				{
					__set_PRIMASK(1);  //�ر��ж�
					NVIC_SystemReset();  //����Ƭ������
				}
			break;
			case 0xE1:  //��ȡ����
				ParID = DT_Rxbuffer[5]<<8 | DT_Rxbuffer[4];  //��ȡ����ID
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //У�鲻ͨ��������
					return;
				bsp_dt_ANO_Send_Parameter(ParID);
			break;
			case 0xE2:  //д�����
				for(int i=0;i<DT_Rxbuffer[3]+4;i++)
				{
					sc += DT_Rxbuffer[i];
					ac += sc;
				}
				ParID = DT_Rxbuffer[5]<<8 | DT_Rxbuffer[4];
				if(DT_Rxbuffer[DT_Rxbuffer[3]+4] !=sc || DT_Rxbuffer[DT_Rxbuffer[3]+5] != ac)  //У�鲻ͨ��������
					return;
				bsp_dt_ANO_Send_Check(0XE2,sc,ac);
				if(ParID >= 11 && ParID <=64)  //����PID�����б�ʱ��Ҫ��С1000���ٸ��µ���Ӧ������
					bsp_dt_ParList[ParID] = (int32_t)(DT_Rxbuffer[6] | DT_Rxbuffer[7]<<8 | DT_Rxbuffer[8]<<16 | DT_Rxbuffer[9]<<24);
				else
					bsp_dt_ParList[ParID] = (int32_t)(DT_Rxbuffer[6] | DT_Rxbuffer[7]<<8 | DT_Rxbuffer[8]<<16 | DT_Rxbuffer[9]<<24);
			break;
			default:
			break;
		}
	}
	else
	{
		return;  //֡ͷ���ݲ��ԣ�����
	}
}

/**
 * @brief ���ݴ����ʼ��
 */
void bsp_dt_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&BSP_DT_UART); //��������ж�λ
	__HAL_UART_ENABLE_IT(&BSP_DT_UART,UART_IT_IDLE); //ʹ��DMA���տ����ж�
	HAL_UART_Receive_DMA(&BSP_DT_UART, (uint8_t*)DT_Rxbuffer, BSP_DT_BUFFER_SIZE); //��ʼDMA����

	//��ʼ���汾������ģ��Ϊ�ؿ��߷ɿأ�����һ���ֹ���û����
	bsp_dt_ParList[BSP_DT_PAR_DEVICE_NAME] = INFANTRY_NAME;
	bsp_dt_ParList[BSP_DT_PAR_HWTYPE] = INFANTRY_HWTYPE;
	bsp_dt_ParList[BSP_DT_PAR_HWVER] = INFANTRY_HWVER;
	bsp_dt_ParList[BSP_DT_PAR_SWVER] = INFANTRY_SWVER;
	bsp_dt_ParList[BSP_DT_PAR_BLVER] = INFANTRY_BLVER;
}

/**
 * @brief ���ݴ����жϴ���
 */
void bsp_dt_It(void)
{
	if(__HAL_UART_GET_FLAG(&BSP_DT_UART,UART_FLAG_IDLE) != RESET)	//��������˿����ж�
	{
		HAL_UART_DMAStop(&BSP_DT_UART); //�ر�DMA
		bsp_dt_Ano_Data_Analysis();
		memset(DT_Rxbuffer,0,BSP_DT_BUFFER_SIZE); //������ɣ�������0
		__HAL_UART_CLEAR_IDLEFLAG(&BSP_DT_UART); //��������жϱ�־λ
		HAL_UART_DMAResume(&BSP_DT_UART);         //���´�DMA�����ǵü�
		HAL_UART_Receive_DMA(&BSP_DT_UART, (uint8_t*)DT_Rxbuffer, BSP_DT_BUFFER_SIZE);//���¿���DMA���մ���
	}
}

/**
 * @brief ��������λ��������Ϣ�ľ������
 * �˺���1MS����һ�Σ��ڲ�ͨ��״̬����ʵ�ֲ�ͬƵ�ʵ����ݴ���
 */
void bsp_dt_ANO_Send_Handle(void)
{
	static uint32_t cnt = 0;
	static uint32_t senser_cnt 	= 10;
	static uint32_t senser2_cnt 	= 50;
	static uint32_t user_cnt 	= 10;
	static uint32_t status_cnt 	= 15;
	static uint32_t rcdata_cnt 	= 20;
	//static uint32_t motopwm_cnt	= 20;
	static uint32_t power_cnt	= 1;
	//static uint32_t speed_cnt   	= 50;
	//static uint32_t location_cnt = 500;

	if((cnt % status_cnt) == (status_cnt-1))
		bsp_dt_ANO_Send_EulerAngle(app_imu_data.Pitch,app_imu_data.Roll,app_imu_data.Yaw);

	if((cnt % senser_cnt) == (senser_cnt-1))
		bsp_dt_ANO_Send_SensorData1(app_imu_data.original.Accel[0],app_imu_data.LPF.Accel[0],app_imu_data.original.Accel[2],
							app_imu_data.original.Gyro[0],app_imu_data.original.Gyro[1],app_imu_data.original.Gyro[2]);

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		bsp_dt_ANO_Send_SensorData2(0,0,0,0,app_imu_data.unitized.MPU_Temp,0,0);

	if((cnt % power_cnt) == (power_cnt-1))
		bsp_dt_ANO_Send_VolatileCurrent(24.0,1.55);

	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		bsp_dt_ANO_Send_Remote(bsp_dbus_Data.CH_0,bsp_dbus_Data.CH_1,bsp_dbus_Data.CH_2,bsp_dbus_Data.CH_3,bsp_dbus_Data.S1,bsp_dbus_Data.S2,bsp_dbus_Data.Dial);

	if((cnt % user_cnt) == (user_cnt-1))
		bsp_dt_ANO_Send_UserData(0XF1,cnt,-cnt,cnt*cnt);

	if(++cnt>1000) cnt = 0;
}

#undef BYTE0
#undef BYTE1
#undef BYTE2
#undef BYTE3
