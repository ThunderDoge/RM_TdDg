//VisionUart v0.1, 2019/11/12 by ThunderDoge
#ifndef	__APP_VISION_UART_H_
#define	__APP_VISION_UART_H_

#define len_ DATA_FRAME_LENGTH
#define UNPACK_BUF_LENGTH 109

#include "bsp_VisionUart.h"
#include "stdlib.h"

//Dataframe Structures
typedef enum{
    //С������32֮���ͨ��
    CMD_GIMBAL_RELATIVE_CONTROL = 0x01,
    CMD_GIMBAL_ABSOLUTE_CONTROL = 0x02,
    CMD_GET_GIMBAL_ABSOLUTE_ANGLE = 0x03,
    CMD_GET_MCU_STATE = 0x04,
    CMD_SHOOT = 0x05,
	CMD_TXRX_TEST = 0xFE
} SerialPortCMD;

typedef struct{
    float           pitch;
    float           yaw;
    unsigned char   speed;
    unsigned char   enemy_distance;
}GimbolRelative_Rx;			//��̨���λ�ÿ�������֡
typedef struct{
    float           pitch;
    float           yaw;
    unsigned char   speed;
}GimbolAbsolute_Rx;			//��̨����λ�ÿ�������֡
typedef struct{
    unsigned char enshoot;
    unsigned char shoot_mode;
}Shooting_Rx;						//��ƿ�������֡
typedef struct{
    unsigned char mode;
}mcuStatus_Tx;					//MCU״̬����֡
typedef struct{
    float pitch;
}GimbolAbsolute_Tx;			//��̨����λ�ô�����֡
typedef struct{
    int32_t Msg_Cnt_Total;
    int32_t Tx_Value;
}TransmitTest_Tx;				//ͨ�Ų�������֡

typedef struct{
    uint8_t LastFuntionWord;
    GimbolRelative_Rx   GimbolRelative_Ctrl;
    GimbolAbsolute_Rx   GimbolAbsolute_Ctrl;
    Shooting_Rx         Shooting_Ctrl;
	TransmitTest_Tx		TxRxTestMsg;
}VisionInfoTypeDef; //��������������ָ���ṹ��

typedef struct{
    uint8_t FrameHead;
    uint8_t FunctionWord;
    uint8_t Data[14];
    uint8_t SumCheck;
    uint8_t FrameTail;
}Dataframe_t;				//����֡�ṹ��

extern VisionInfoTypeDef VisionInfo;							//����������Ϣ����ṹ��

HAL_StatusTypeDef app_VisionUart_RecvPack(void);		//С������Ϣ����
HAL_StatusTypeDef app_VisionUart_SendPack(uint8_t *Data, uint8_t FunctionWord);  //��С������������֡
HAL_StatusTypeDef app_VisionUart_Send_gimbal_Absolute(float pitch);		//��С����������̨���ԽǶ�
HAL_StatusTypeDef app_VisionUart_Send_mcu_state(uint8_t mode);				//��С��������MCU״̬
HAL_StatusTypeDef app_VisionUart_AckMsg(void);			//С������ϢӦ��
void app_VisionUart_TxTest(int32_t TxMsgCnt);				//MCU����ͨ�Ų���
//HAL_StatusTypeDef app_VisionUart_Send_gimbal_Relative(float pitch,float yaw,unsigned char gimbal_speed_mode,unsigned char shoot_mode);	//��С����������̨��ԽǶȡ��Ժ���е�

#endif	//__APP_VISION_UART_H_
