/**
 * @file SentryCloudVision.cpp
  * @brief     云台视觉串口通信
  * @details  提供一个云台与他的小主机进行UART通讯的所有函数。包含接受到的信息：处理-储存，将要发送的信息：打包，发送。
  * 仅仅依赖于app_vision。注意app_vision。这意味着她不能够主动调用其他的组件的函数。所以你将看到很多的**回调函数**的写法
  * @author   ThunderDoge
  * @date     2020-2-20
  * @version  0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
#include "SentryCloudVision.hpp"

static void CloudVision_HandleFunctionWordTransmit(uint8_t func_word);


QueueHandle_t CloudVision_QueueOfMsgToSend;  /// 待发送消息的队列

void CloudVision_TaskTransmit_Init(void)
{
    CloudVision_QueueOfMsgToSend = xQueueCreate(20,sizeof(uint8_t));
}

static void CloudVision_HandleFuncWord(uint8_t func_word);

void CloudVision_TaskTransmit_Handle(void)
{
    uint8_t _func_word;

    if(xQueueReceive(CloudVision_QueueOfMsgToSend,&_func_word,portMAX_DELAY)==pdTRUE)	// 设定为阻塞容许时间为 portMAX_DELAY 来允许空队列时无限阻塞。
    {
        CloudVision_HandleFunctionWordTransmit(_func_word);
    }

    return;    
}
void CloudVision_Transmit_ISR(uint8_t _Functionword);
void CloudVision_Transmit_Emergency_ISR(uint8_t _Functionword);

static void CloudVision_HandleFunctionWordTransmit(uint8_t func_word)
{
    switch (func_word) 
    {
        case CMD_GET_MCU_STATE:
		
		    VisionTx.Cloud_mode = CloudEntity.Mode;
			VisionTx.Shoot_mode = CloudEntity.shoot_flag;
			VisionTx.Pitch = CloudEntity.RealPitch;
			VisionTx.YawSoft = CloudEntity.RealYaw;
			VisionTx.Yaw = CloudEntity.MechanicYaw;
			VisionTx.Shoot_speed = 0;	// 未实现
			CMD_GET_MCU_STATE_Tx(VisionTx.Pitch, VisionTx.Yaw, VisionTx.YawSoft, VisionTx.Cloud_mode, VisionTx.Shoot_mode);
			break;
			
		case ROBOT_ERR:
		
			VisionTx.Error_code = 0;	// 未实现
			    ROBOT_ERR_Tx(VisionTx.Error_code);
			break;
			
		case STA_CHASSIS:
		
			VisionTx.chassis_mode = VisionRx.chassis_mode;
			    break;VisionTx.Vx = CanRx.Chassis_SpeedLocation[0];
			VisionTx.pillar_flag = CanRx.Pillar_flag;
			VisionTx.Px = CanRx.Chassis_SpeedLocation[1];
			STA_CHASSIS_Tx(VisionTx.chassis_mode, VisionTx.pillar_flag, VisionTx.Vx, VisionTx.Px);
			break;
		
/*
        case JUD_GAME_STATUS:
        
            JUD_GAME_STATUS_Tx(uint8_t game_progress, uint16_t stage_remain_time);
            break;
        case JUD_ENY_HP:
        
            JUD_ENY_HP_Tx(uint16_t hp);
            break;
        case JUD_GAME_EVENT:
        
            JUD_GAME_EVENT_Tx(); // 待定
            break;
        case JUD_SELF_HP:
        
            JUD_SELF_HP_Tx(uint16_t hp);
            break;
        case JUD_GUN_CHASSIS:
        
            JUD_GUN_CHASSIS_HEAT_Tx(float chassis_power, uint16_t cha_pwr_buf,uint16_t gun_heat);      
            break;
        case JUD_SELF_BUFF:
        
            JUD_SELF_BUFF_Tx(uint8_t buff_code);
            break;
        case JUD_TAKING_DMG:
        
            JUD_TAKING_DMG_Tx(uint8_t armor_id_enum, uint8_t hurt_type_enum);
            break;
        case JUD_SHOOTING:
        
            JUD_SHOOTING_Tx(uint8_t bullet_freq, float bullet_speed);
            break;
        case JUD_AMMO_LEFT:
        
            JUD_AMMO_LEFT_Tx(uint16_t bulelt_left);
            break;
        case JUD_USER:
        
            JUD_USER_Tx();
			break;
*/
	}
}














// #define DEBUG

// void CloudVisionRoutine(void)
// {
// 	VisionTx.Pitch  = CloudEntity.RealPitch;
// 	VisionTx.Yaw    = CloudEntity.MechanicYaw;

// 	VisionTx.Ready_flag = 1;
// 	app_vision_SendData((uint8_t)CMD_GET_MCU_STATE);
// 	#ifndef DEBUG
// 	vTaskDelay(1);
//     VisionTx.pillar_flag = CloudCanRecv.pillar_close_flag;
//     VisionTx.Px = CloudCanRecv.ChassisLocation;
//     app_vision_SendData((uint8_t)STA_CHASSIS);
// 	#endif
// }
#undef DEBUG 



