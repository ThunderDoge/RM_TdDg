#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"

//�궨���õ���CAN
#define USE_CAN1
#define USE_CAN2

#define CLOUD_TO_CHASSIS_ID  0x101			//����CANͨ��ID

extern CAN_TxHeaderTypeDef Bsp_CAN1_Tx;   /*Can���߷���*/
extern CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can���߽���*/
extern CAN_TxHeaderTypeDef Bsp_CAN2_Tx;   /*Can2���߷���*/
extern CAN_RxHeaderTypeDef Bsp_CAN2_Rx;		/*Can2���߽���*/


extern uint8_t CAN1_RxData[8];       /*CAN1���ջ�������*/
extern uint8_t CAN1_TxData[8];   /*CAN1��������*/
extern uint8_t CAN2_RxData[8];       /*CAN2���ջ�������*/
extern uint8_t CAN2_TxData[8];   /*CAN2��������*/
extern int16_t CAN1_TxData16[4];
extern int16_t CAN2_TxData16[4];


extern int16_t CAN1_ReLoc; /*CAN���߽��յ�λ������*/
extern int16_t CAN1_ReRpm; /*CAN���߽��յ�ת������*/

typedef struct{
		uint8_t SuperCapFlag :4;//�������ݵ�ʹ�ÿ��� 1Ϊʹ��
		uint8_t ReverseFlag :1;//���̷����־ 1Ϊ����
		uint8_t SCIAFlag :1;//���ݶ���������̱�־ 1Ϊ��������
		uint8_t AutoReloadCalibrationFlag : 1;//�Զ���λУ׼��־
		uint8_t CloudJudgementOutOfContact : 1;//��̨����ϵͳʧ��
//		uint8_t LaserRadarMode_ON : 1;//���ü����״�ģʽ��־
}RecFlags_t;//�����־


typedef struct//��(CLOUD_TO_CHASSIS_ID)
{
	uint8_t cmd;//�����˶�ģʽ
	RecFlags_t RecFlags;//���ֱ�־��1��Ч��[0-3]�������ݣ�[4]���̷���
	int16_t speed_x;
	int16_t speed_y;
	int16_t speed_z;
}Chassis_Control_Data_t;
extern Chassis_Control_Data_t Chassis_Control_Data;//��̨���صĿ�����Ϣ


/*����������ݽṹ��*/
typedef struct can_rec
{
	int16_t RealPosition;	    //!< ��ʵλ��(������)
	int16_t LastPosition;     //!< �ϴ�λ��
	int16_t TargetPosition;	  //!< Ŀ��λ��
	int16_t RealSpeed;		    //!< ʵ���ٶ�(������)
	int16_t TargetSpeed;	    //!< Ŀ���ٶ�
	int16_t RealCurrent;      //!< ��ʵ����(������)
	int16_t TargetCurrent;  	//!< Ŀ�����
	uint8_t RealTemperature;
	uint32_t StartUpdataTime;
}Motor_Data;


extern Motor_Data Motor_Yaw_Data; 	//ȫ�ֵ������
extern Motor_Data Motor_Pitch_Data;
extern Motor_Data Motor_1_Data;
extern Motor_Data Motor_2_Data;
extern Motor_Data Motor_3_Data;
extern Motor_Data Motor_4_Data;


void bsp_CAN_Init(void);
void bsp_CAN_TxLoadAllMotorData(void);
void bsp_CAN_Sendmessage(void);
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff);
void bsp_CAN_Analysis(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
