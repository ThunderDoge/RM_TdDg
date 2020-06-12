/**
* @file bsp_judgement.c
* @brief 裁判系统驱动2020
* @author OnePointFive
* @date 2020.04.30
* @version 1.0
* @copyright Copyright (c) RM2020电控
* @par 日志:
*		v1.0 移植2019年驱动并按照2020新协议做了更改
*/

#include "bsp_judgement.h"
#include <string.h>
#include <stdio.h>


uint32_t JudgeMsgRxTick;	/// 更新时间戳


#define	CRC_Check	//是否要使用CRC校验		反正都是大疆写的那就用咯

#ifdef CRC_Check
/*----------------------CRC8 Check Sum Part---------------------------*/
static const unsigned char CRC8_INIT = 0xff;
static const unsigned char CRC8_TAB[256] =
{
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
		unsigned char ucIndex;
		while (dwLength--)
		{
			ucIndex = ucCRC8^(*pchMessage++);
			ucCRC8 = CRC8_TAB[ucIndex];
		}
		return(ucCRC8);
}

/**
* @brief CRC8确认校验和
* @param  pchMessage
* @param  dwLength
* @return unsigned int 
*/
static unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
		unsigned char ucExpected = 0;
		if ((pchMessage == 0) || (dwLength <= 2)) 
			return 0;
		ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
		return ( ucExpected == pchMessage[dwLength-1] );
}

/**
* @brief CRC8添加校验和	
* @param  pchMessage
* @param  dwLength
*/
static void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) 
		return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}

/*----------------------CRC16 Check Sum Part---------------------------*/

static uint16_t CRC_INIT = 0xffff;

static const uint16_t wCRC_Table[256] =
{
		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
		0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
		0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
		0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
		0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
		0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
		0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
		0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
		0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
		0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
		0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
		0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
		0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
		0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
		0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
		0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	 	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
		0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
		0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
* @brief CRC16 checksum function
* @param  pchMessage
* @param  dwLength
* @param  wCRC    
* @return uint16_t 
*/
static uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

/**
* @brief CRC16确认校验和	
* @param  pchMessage
* @param  dwLength
* @return uint32_t 
*/
static uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
		uint16_t wExpected = 0;
		if ((pchMessage == NULL) || (dwLength <= 2))
		{
			return 0xFFFF;
		}
		wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
		return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
* @brief CRC16添加校验和	
* @param  pchMessage
* @param  dwLength
*/
static void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}
	wCRC = Get_CRC16_Check_Sum ( (uint8_t  *)pchMessage, dwLength-2, CRC_INIT );
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

#endif

///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\
// * @ Brief: 串口复位重启DMA										           *
// * @ Param: RC_Channel RC_Buff											   *
// * @ Attention: 串口中断函数处理完后调用	 							   *
// * @ Author&Date:     MushroomChan        2017/4/13						   *
//\* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//static void uart_reset_idle_rx_callback(UART_HandleTypeDef *huart)
//{
//	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(huart);
//		//clear idle it flag
//		//重启DMA
//		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
//			
//		__HAL_DMA_DISABLE(huart->hdmarx);
//		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,	DMA_FLAGS);
////		__HAL_DMA_SET_COUNTER(huart->hdmarx,	MAX_DMA_COUNT);
//		__HAL_DMA_ENABLE(huart->hdmarx);
//	}
//}

///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\
// * @ Brief: 串口复位串口中断标志位								           *
// * @ Param: RC_Channel RC_Buff											   *
// * @ Attention: 串口中断函数处理完后调用	 							   *
// * @ Author&Date:     MushroomChan        2017/4/13						   *
//\* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//static void uart_reset_uartIT(UART_HandleTypeDef *huart)
//{
//	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(huart);
//	}
//}



/**
* @brief int16_t限幅
* @param  out     
* @param  min     
* @param  max     
* @return int16_t 
*/
static int16_t limit_short(int16_t out, int16_t min, int16_t max)
{
	if(out > max)
		return max;
	else if(out < min)
		return min;
	else return out;
}

///////////////////////////////////////////////////////////////////////////////



#define MAX_DMA_COUNT	100
HAL_StatusTypeDef JudgeMentOnLineStatus;

//接收数据包
ext_game_status_t game_status;//比赛状态（接收）
ext_game_result_t game_result;//比赛结果数据（接收）
ext_game_robot_HP_t game_robot_HP;//比赛机器人血量数据（接收）
ext_dart_status_t dart_status;//飞镖发射状态（接收）
ext_event_data_t event_data;//场地事件数据（接收）
ext_supply_projectile_action_t supply_projectile_action;//场地补给站动作标识数据（接收）
ext_referee_warning_t referee_warning;//裁判警告数据（接收）
ext_dart_remaining_time_t dart_remaining_time;//飞镖发射口倒计时（接收）
ext_game_robot_status_t game_robot_status;//机器人状态数据（接收）
ext_power_heat_data_t power_heat_data;//实时功率热量数据（接收）
ext_game_robot_pos_t game_robot_pos;//机器人位置数据（接收）
ext_buff_t buff_musk;//机器人增益数据（接收）
ext_aerial_robot_energy_t aerial_robot_energy;//空中机器人能量状态数据（接收）
ext_robot_hurt_t robot_hurt;//伤害状态数据（接收）
ext_shoot_data_t shoot_data;//实时射击数据（接收）
ext_bullet_remaining_t bullet_remaining;//子弹剩余发射数（接收）
ext_rfid_status_t rfid_status;//机器人 RFID 状态（接收）
robot_interactive_dataPack_t robot_interactive_dataPack_r;	//接收机间交互数据包

//发送数据包
client_custom_dataPack_t client_custom_dataPack;					//客户端自定义数据包					//子弹预约数据包
robot_interactive_dataPack_t robot_interactive_dataPack_s;//发送机间交互数据
client_graphic_dataPack_t client_graphic_dataPack;//客户端图形自定义数据包

static uint8_t bsp_judgement_data[400];
static uint32_t LastJudgePackTick;
uint32_t JudgePeriod;			//debug看接收周期用
static int16_t limit_short(int16_t out, int16_t min, int16_t max);

/**
* @brief 裁判系统串口初始化
*/
void bsp_judgement_Init(void)
{
	__HAL_UART_ENABLE_IT(&BSP_JUDGEMENT_UART, UART_IT_TC);
	__HAL_UART_ENABLE_IT(&BSP_JUDGEMENT_UART, UART_IT_IDLE);
 	HAL_UART_Receive_DMA(&BSP_JUDGEMENT_UART, bsp_judgement_data, 200);
}

/**
* @brief 裁判系统接收数据解算
*/
static void bsp_judgement_Calculate(void)
{
	uint8_t index = 0;
	uint8_t Back1PackFlag = 0;
	frame_header_t Header;
	if(bsp_judgement_data[0] == 0xA5 && bsp_judgement_data[1] == 0xA5)
	{
		Back1PackFlag = 1;//说明此包向后移了一位
		index = 1;
	}
	memcpy(&Header,bsp_judgement_data + index,sizeof(Header));
	index += sizeof(Header);
		
	if((Header.SOF == 0xA5)
		&& 	(Verify_CRC8_Check_Sum(bsp_judgement_data + Back1PackFlag,sizeof(Header)) == 1)
		&&	(Verify_CRC16_Check_Sum(bsp_judgement_data + Back1PackFlag,Header.DataLength + 9) == 1)	
	  )			
	{
		JudgeMsgRxTick = HAL_GetTick(); // 更新设备接收时间

		while(index  <= 199)
		{
			uint32_t now = HAL_GetTick();
			JudgePeriod = now - LastJudgePackTick;
			LastJudgePackTick = now;
			uint16_t CmdID = 0;//获取命令码类型
			memcpy(&CmdID,(bsp_judgement_data+index),sizeof(CmdID));
			index+=2;
			
			switch(CmdID)
			{
				case ext_game_status_id://0x0001//3 比赛状态数据，1Hz 周期发送
					memcpy(&game_status,bsp_judgement_data+index,sizeof(game_status));
					index+=sizeof(game_status);
				break;
				case ext_game_result_id://0x0002//1 比赛结果数据，比赛结束后发送
					memcpy(&game_result,bsp_judgement_data+index,sizeof(game_result));
					index+=sizeof(game_result);
				break;
				case ext_game_robot_HP_id ://0x0003//32 比赛机器人血量数据，1Hz 发送
					memcpy(&game_robot_HP,bsp_judgement_data+index,sizeof(game_robot_HP));
					index+=sizeof(game_robot_HP);
				break;
				case ext_dart_status_id ://0x0004//3 飞镖发射状态，飞镖发射时发送
					memcpy(&dart_status,bsp_judgement_data+index,sizeof(dart_status));
					index+=sizeof(dart_status);
				break;
				case ext_event_data_id://0x0101//4 场地事件数据，1Hz 周期发送
					memcpy(&event_data,bsp_judgement_data+index,sizeof(event_data));
					index+=sizeof(event_data);
				break;
				case ext_supply_projectile_action_id://0x0102//4 场地补给站动作标识数据，动作改变后发送
					memcpy(&supply_projectile_action,bsp_judgement_data+index,sizeof(supply_projectile_action));
					index+=sizeof(supply_projectile_action);
				break;
				case ext_referee_warning_id://0x0104//2 裁判警告数据，警告发生后发送
					memcpy(&referee_warning,bsp_judgement_data+index,sizeof(referee_warning));
					index+=sizeof(referee_warning);
				break;
				case ext_dart_remaining_time_id://0x0105//1 飞镖发射口倒计时，1Hz 周期发送
					memcpy(&dart_remaining_time,bsp_judgement_data+index,sizeof(dart_remaining_time));
					index+=sizeof(dart_remaining_time);
				break;
				case ext_game_robot_status_id://0x0201//18 机器人状态数据，10Hz 周期发送
					memcpy(&game_robot_status,bsp_judgement_data+index,sizeof(game_robot_status));
					index+=sizeof(game_robot_status);
				break;
				case ext_power_heat_data_id://0x0202 //16 实时功率热量数据，50Hz 周期发送
					memcpy(&power_heat_data,bsp_judgement_data+index,sizeof(power_heat_data));
					index+=sizeof(power_heat_data);
				break;
				case ext_game_robot_pos_id://0x0203//16 机器人位置数据，10Hz 发送
					memcpy(&game_robot_pos,bsp_judgement_data+index,sizeof(game_robot_pos));
					index+=sizeof(game_robot_pos);
				break;
				case ext_buff_id://0x0204//1 机器人增益数据，1Hz 周期发送
					memcpy(&buff_musk,bsp_judgement_data+index,sizeof(buff_musk));
					index+=sizeof(buff_musk);
				break;
				case ext_aerial_robot_energy_id://0x0205//3 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
					memcpy(&aerial_robot_energy,bsp_judgement_data+index,sizeof(aerial_robot_energy));
					index+=sizeof(aerial_robot_energy);
				break;
				case ext_robot_hurt_id://0x0206//1 伤害状态数据，伤害发生后发送
					memcpy(&robot_hurt,bsp_judgement_data+index,sizeof(robot_hurt));
					index+=sizeof(robot_hurt);
				break;
				case ext_shoot_data_id://0x0207//6 实时射击数据，子弹发射后发送
					memcpy(&shoot_data,bsp_judgement_data+index,sizeof(shoot_data));
					index+=sizeof(shoot_data);
				break;
				case ext_bullet_remaining_id://0x0208//2 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人
					memcpy(&bullet_remaining,bsp_judgement_data+index,sizeof(bullet_remaining));
					index+=sizeof(bullet_remaining);
				break;
				case ext_rfid_status_id://0x0209//4 机器人 RFID 状态，1Hz 周期发送
					memcpy(&rfid_status,bsp_judgement_data+index,sizeof(rfid_status));
					index+=sizeof(rfid_status);
				break;
				case ext_student_interactive_header_data_id://0x0301 //n 机器人间交互数据，发送方触发发送，上限 10Hz
					memcpy(&robot_interactive_dataPack_r, bsp_judgement_data, sizeof(robot_interactive_dataPack_r));
					index+=sizeof(robot_interactive_dataPack_r);
				break;
				default:break;
			}
			index+=7;
		}
		memset(bsp_judgement_data, 0, 237);
	}
}	

/**
* @brief 裁判系统中断函数，需要放入对应串口Handle
*/
void bsp_judgement_It(void)
{
	if(__HAL_UART_GET_FLAG(&BSP_JUDGEMENT_UART,UART_FLAG_IDLE))
	{
		HAL_UART_DMAStop(&BSP_JUDGEMENT_UART);
		bsp_judgement_Calculate();
		__HAL_UART_CLEAR_IDLEFLAG(&BSP_JUDGEMENT_UART);
		HAL_UART_Receive_DMA(&BSP_JUDGEMENT_UART,(uint8_t*)bsp_judgement_data,200);
	}
}

static uint8_t Judge_SendBuff[128];
/**
* @brief 发送自定义数据到裁判系统给上位机显示
* @details 调用该函数前先将client_custom_dataPack.client_custom_data赋值
* @param  data1   
* @param  data2   
* @param  data3   
* @param  singleLight
*/
void bsp_judgement_SendCustomData(float data1,float data2,float data3,uint8_t singleLight)
{
	client_custom_dataPack.client_custom_data.data1 = data1;
	client_custom_dataPack.client_custom_data.data2 = data2;
	client_custom_dataPack.client_custom_data.data3 = data3;
	client_custom_dataPack.client_custom_data.masks = singleLight;

	//帧头
	client_custom_dataPack.Header_Tx.SOF = 0xA5;
	client_custom_dataPack.Header_Tx.DataLength = 19;//6bytes + client_custom_data:13bytes
	Append_CRC8_Check_Sum((uint8_t*)&client_custom_dataPack.Header_Tx, sizeof(frame_header_t));
	//命令码ID
	client_custom_dataPack.CmdID_Tx = ext_student_interactive_header_data_id;
	//数据的内容ID
	client_custom_dataPack.student_interactive_header_data.data_cmd_id = 0xd180;
	//发送者的 ID
	client_custom_dataPack.student_interactive_header_data.sender_ID = game_robot_status.robot_id;
	//客户端 ID
	if(game_robot_status.robot_id < 10)//红方
		client_custom_dataPack.student_interactive_header_data.receiver_ID = game_robot_status.robot_id+0x100;
	else 
		client_custom_dataPack.student_interactive_header_data.receiver_ID = game_robot_status.robot_id+0x106;
	//整包校验
	Append_CRC16_Check_Sum((uint8_t*)&client_custom_dataPack, sizeof(client_custom_dataPack));
	//复制到缓冲池
	memcpy(Judge_SendBuff, &client_custom_dataPack, sizeof(client_custom_dataPack));
	//发送
	HAL_UART_Transmit_DMA(&BSP_JUDGEMENT_UART, Judge_SendBuff, sizeof(client_custom_dataPack));
}

/**
* @func 	发送自定义交互数据到裁判系统用于交互
* @note		最高频率10Hz
					调用该函数前先将robot_interactive_dataPack_s.robot_interactive_data.data赋值
					data_id 范围：0x200~0x2ff
**/

/**
* @brief 发送自定义交互数据到裁判系统用于交互
* @details	调用该函数前先将robot_interactive_dataPack_s.robot_interactive_data.data赋值
*			data_id 范围：0x200~0x2ff
* @param  receiver_index
* @param  data_id 
*/
void bsp_judgement_SendInteractiveData(uint8_t receiver_index, uint16_t data_id)
{
	//帧头
	robot_interactive_dataPack_s.Header_Tx.SOF = 0xA5;
	robot_interactive_dataPack_s.Header_Tx.DataLength = 61;//6bytes + robot_interactive_data
	Append_CRC8_Check_Sum((uint8_t*)&robot_interactive_dataPack_s.Header_Tx, sizeof(frame_header_t));
	//命令码ID
	robot_interactive_dataPack_s.CmdID_Tx = ext_student_interactive_header_data_id;
	//数据的内容ID
	robot_interactive_dataPack_s.student_interactive_header_data.data_cmd_id = data_id;
	//发送者的 ID 
	robot_interactive_dataPack_s.student_interactive_header_data.sender_ID = game_robot_status.robot_id;
	//接受者 ID
	if(game_robot_status.robot_id < 10)//红方
		robot_interactive_dataPack_s.student_interactive_header_data.receiver_ID = receiver_index;
	else robot_interactive_dataPack_s.student_interactive_header_data.receiver_ID = receiver_index +10;
	//整包校验
	Append_CRC16_Check_Sum((uint8_t*)&robot_interactive_dataPack_s, sizeof(robot_interactive_dataPack_s));
	//复制到缓冲池
	memcpy(Judge_SendBuff, &robot_interactive_dataPack_s, sizeof(robot_interactive_dataPack_s));
	//发送
	HAL_UART_Transmit_DMA(&BSP_JUDGEMENT_UART, Judge_SendBuff, sizeof(robot_interactive_dataPack_s));
}
/**
* @func 	发送自定义图形交互数据到裁判系统用于交互
* @note		最高频率10Hz
					调用该函数前先将client_custom_dataPack.client_graphic_draw赋值
					data_id 范围：0x200~0x2ff
**/

/**
* @brief 发送自定义图形交互数据到裁判系统用于交互
* @details 调用该函数前先将client_custom_dataPack.client_graphic_draw赋值
*		   data_id 范围：0x200~0x2ff
*/
void bsp_judgement_SendGraphicData(void)
{
	//帧头
	client_graphic_dataPack.Header_Tx.SOF = 0xA5;
	client_graphic_dataPack.Header_Tx.DataLength = 61;//6bytes + client_graphic_draw:55bytes
	Append_CRC8_Check_Sum((uint8_t*)&client_graphic_dataPack.Header_Tx, sizeof(frame_header_t));
	//命令码ID
	client_graphic_dataPack.CmdID_Tx = ext_student_interactive_header_data_id;
	//数据的内容ID
	client_graphic_dataPack.student_interactive_header_data.data_cmd_id = 0x0100;
	//发送者的 ID
	client_graphic_dataPack.student_interactive_header_data.sender_ID = game_robot_status.robot_id;
	//客户端 ID
	if(game_robot_status.robot_id < 10)//红方
		client_graphic_dataPack.student_interactive_header_data.receiver_ID = game_robot_status.robot_id+0x100;
	else client_graphic_dataPack.student_interactive_header_data.receiver_ID = game_robot_status.robot_id+0x106;
	//整包校验
	Append_CRC16_Check_Sum((uint8_t*)&client_graphic_dataPack, sizeof(client_graphic_dataPack));
	//复制到缓冲池
	memcpy(Judge_SendBuff, &client_graphic_dataPack, sizeof(client_graphic_dataPack));
	//发送
	HAL_UART_Transmit_DMA(&BSP_JUDGEMENT_UART, Judge_SendBuff, sizeof(client_graphic_dataPack));
}

/**
* @brief 在指定位置添加/修改一条线
* @param  name    图形索引
* @param  operation 操作
* @param  layer   图层(0~9)
* @param  x_start x轴起始位置(0~1920)
* @param  y_start y轴起始位置(0~1080)
* @param  x_end   x轴终点位置(0~1920)
* @param  y_end   y轴终点位置(0~1080)
* @param  color   color 图形颜色(0~8)
* @param  line_width 线宽(单位像素点)
*/
void Client_Line(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end,
	uint8_t color, uint8_t line_width)
{
	sprintf((char*)client_graphic_dataPack.client_graphic_draw.graphic_name, name);
	client_graphic_dataPack.client_graphic_draw.layer = layer;
	client_graphic_dataPack.client_graphic_draw.operate_tpye = operation;
	client_graphic_dataPack.client_graphic_draw.graphic_tpye = Line;
	client_graphic_dataPack.client_graphic_draw.start_x = limit_short(x_start, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.start_y = limit_short(y_start, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.end_x = limit_short(x_end, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.end_y = limit_short(y_end, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.width = line_width;
	client_graphic_dataPack.client_graphic_draw.color = color;
}


/**
* @brief 在指定位置添加/修改一个矩形
* @param  name    图形索引
* @param  operation 操作
* @param  layer   图层(0~9)
* @param  x_start x轴起始位置(0~1920)
* @param  y_start y轴起始位置(0~1080)
* @param  x_length x轴边长(0~1920)
* @param  y_length y轴边长(0~1080)
* @param  color   图形颜色(0~8)
* @param  line_width 线宽(单位像素点)
*/
void Client_Rctangle(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_length, uint16_t y_length, 
	uint8_t color, uint8_t line_width)
{
	sprintf((char*)client_graphic_dataPack.client_graphic_draw.graphic_name, name);
	client_graphic_dataPack.client_graphic_draw.layer = layer;
	client_graphic_dataPack.client_graphic_draw.operate_tpye = operation;
	client_graphic_dataPack.client_graphic_draw.graphic_tpye = Rctangle;
	client_graphic_dataPack.client_graphic_draw.start_x = limit_short(x_start, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.start_y = limit_short(y_start, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.end_x = limit_short(x_start+x_length, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.end_y = limit_short(y_start+y_length, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.width = line_width;
	client_graphic_dataPack.client_graphic_draw.color = color;
}


/**
* @brief 在指定位置添加/修改一个圆
* @param  name    图形索引
* @param  operation 操作
* @param  layer   图层(0~9)
* @param  x_start 圆心横坐标(0~1920)
* @param  y_start 圆心纵坐标(0~1080)
* @param  radius  圆半径(0~540)
* @param  color   图形颜色(0~8)
* @param  line_width 线宽(单位像素点)
*/
void Client_Circle(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t radius,
	uint8_t color, uint8_t line_width)
{
	sprintf((char*)client_graphic_dataPack.client_graphic_draw.graphic_name, name);
	client_graphic_dataPack.client_graphic_draw.layer = layer;
	client_graphic_dataPack.client_graphic_draw.operate_tpye = operation;
	client_graphic_dataPack.client_graphic_draw.graphic_tpye = Circle;
	client_graphic_dataPack.client_graphic_draw.start_x = limit_short(x_start, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.start_y = limit_short(y_start, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.radius = radius;
	client_graphic_dataPack.client_graphic_draw.width = line_width;
	client_graphic_dataPack.client_graphic_draw.color = color;
}
/**
* @brief 在指定位置添加/修改一个椭圆
* @param  name    
* @param  operation
* @param  layer   
* @param  x_start 
* @param  y_start 
* @param  x_half_axis
* @param  y_half_axis
* @param  color   
* @param  line_width
*/
void Client_Ellipse(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_half_axis,
	uint16_t y_half_axis, uint8_t color, uint8_t line_width)
{
	sprintf((char*)client_graphic_dataPack.client_graphic_draw.graphic_name, name);
	client_graphic_dataPack.client_graphic_draw.layer = layer;
	client_graphic_dataPack.client_graphic_draw.operate_tpye = operation;
	client_graphic_dataPack.client_graphic_draw.graphic_tpye = Ellipse;
	client_graphic_dataPack.client_graphic_draw.start_x = limit_short(x_start, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.start_y = limit_short(y_start, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.end_x = x_half_axis;
	client_graphic_dataPack.client_graphic_draw.end_y = y_half_axis;
	client_graphic_dataPack.client_graphic_draw.width = line_width;
	client_graphic_dataPack.client_graphic_draw.color = color;	
}

/**
* @brief 在指定位置添加/修改一个圆弧
* @param  name    图形索引
* @param  operation 操作
* @param  layer   图层(0~9)
* @param  x_start x轴起始位置(0~1920)
* @param  y_start y轴起始位置(0~1080)
* @param  half_axis_length 半轴长度
* @param  start_angle 起始角度
* @param  end_angle 终止角度
* @param  color   图形颜色(0~8)
* @param  line_width 线宽(单位像素点)
*/
void Client_Arc(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t half_axis_length,
	int16_t start_angle, int16_t end_angle, uint8_t color, uint8_t line_width)
{
	sprintf((char*)client_graphic_dataPack.client_graphic_draw.graphic_name, name);
	client_graphic_dataPack.client_graphic_draw.layer = layer;
	client_graphic_dataPack.client_graphic_draw.operate_tpye = operation;
	client_graphic_dataPack.client_graphic_draw.graphic_tpye = Arc;
	client_graphic_dataPack.client_graphic_draw.start_angle = start_angle;
	client_graphic_dataPack.client_graphic_draw.end_angle = end_angle;
	client_graphic_dataPack.client_graphic_draw.start_x = limit_short(x_start, 0, 1920);
	client_graphic_dataPack.client_graphic_draw.start_y = limit_short(y_start, 0, 1080);
	client_graphic_dataPack.client_graphic_draw.end_x = half_axis_length;
	client_graphic_dataPack.client_graphic_draw.end_y = half_axis_length;
	client_graphic_dataPack.client_graphic_draw.width = line_width;
	client_graphic_dataPack.client_graphic_draw.color = color;
}


