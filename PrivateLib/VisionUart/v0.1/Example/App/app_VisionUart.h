//VisionUart v0.1, 2019/11/12 by ThunderDoge
#ifndef	__APP_VISION_UART_H_
#define	__APP_VISION_UART_H_

#define len_ DATA_FRAME_LENGTH
#define UNPACK_BUF_LENGTH 109

#include "bsp_VisionUart.h"
#include "stdlib.h"

//Dataframe Structures
typedef enum{
    //小主机和32之间的通信
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
}GimbolRelative_Rx;			//云台相对位置控制数据帧
typedef struct{
    float           pitch;
    float           yaw;
    unsigned char   speed;
}GimbolAbsolute_Rx;			//云台绝对位置控制数据帧
typedef struct{
    unsigned char enshoot;
    unsigned char shoot_mode;
}Shooting_Rx;						//设计控制数据帧
typedef struct{
    unsigned char mode;
}mcuStatus_Tx;					//MCU状态数据帧
typedef struct{
    float pitch;
}GimbolAbsolute_Tx;			//云台绝对位置答复数据帧
typedef struct{
    int32_t Msg_Cnt_Total;
    int32_t Tx_Value;
}TransmitTest_Tx;				//通信测试数据帧

typedef struct{
    uint8_t LastFuntionWord;
    GimbolRelative_Rx   GimbolRelative_Ctrl;
    GimbolAbsolute_Rx   GimbolAbsolute_Ctrl;
    Shooting_Rx         Shooting_Ctrl;
	TransmitTest_Tx		TxRxTestMsg;
}VisionInfoTypeDef; //控制器接收主机指令缓存结构体

typedef struct{
    uint8_t FrameHead;
    uint8_t FunctionWord;
    uint8_t Data[14];
    uint8_t SumCheck;
    uint8_t FrameTail;
}Dataframe_t;				//数据帧结构体

extern VisionInfoTypeDef VisionInfo;							//解析到的消息储存结构体

HAL_StatusTypeDef app_VisionUart_RecvPack(void);		//小主机信息接收
HAL_StatusTypeDef app_VisionUart_SendPack(uint8_t *Data, uint8_t FunctionWord);  //向小主机发送数据帧
HAL_StatusTypeDef app_VisionUart_Send_gimbal_Absolute(float pitch);		//向小主机发送云台绝对角度
HAL_StatusTypeDef app_VisionUart_Send_mcu_state(uint8_t mode);				//像小主机发送MCU状态
HAL_StatusTypeDef app_VisionUart_AckMsg(void);			//小主机信息应答
void app_VisionUart_TxTest(int32_t TxMsgCnt);				//MCU发起通信测试
//HAL_StatusTypeDef app_VisionUart_Send_gimbal_Relative(float pitch,float yaw,unsigned char gimbal_speed_mode,unsigned char shoot_mode);	//向小主机发送云台相对角度。以后会有的

#endif	//__APP_VISION_UART_H_
