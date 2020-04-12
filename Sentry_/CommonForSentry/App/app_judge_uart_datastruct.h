/**
  * @file      app_judge_uart_datastruct.h
  * @brief     裁判系统数据解析结构 集合
  * @details   
  * @author   ThunderDoge
  * @date      
  * @version   
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                           Using encoding: gb2312
  */
#ifndef __APP_JUDGE_UART_DATASTRUCT_H
#define __APP_JUDGE_UART_DATASTRUCT_H

#include "stm32f4xx.h"


typedef __packed struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}FrameHead;

typedef __packed struct 
{ 
    uint8_t game_type : 4; 
    uint8_t game_progress : 4; 
    uint16_t stage_remain_time; 
} ext_game_status_t;

typedef __packed struct 
{ 
      uint8_t winner; 
} ext_game_result_t; 

typedef __packed struct 
{ 
    uint16_t red_1_robot_HP;   
    uint16_t red_2_robot_HP;   
    uint16_t red_3_robot_HP;   
    uint16_t red_4_robot_HP;   
    uint16_t red_5_robot_HP;   
    uint16_t red_7_robot_HP;   
    uint16_t red_base_HP;   
    uint16_t blue_1_robot_HP;   
    uint16_t blue_2_robot_HP;   
    uint16_t blue_3_robot_HP;   
    uint16_t blue_4_robot_HP;   
    uint16_t blue_5_robot_HP;   
    uint16_t blue_7_robot_HP;   
    uint16_t blue_base_HP; 
} ext_game_robot_HP_t; 

typedef __packed struct 
{ 
    uint32_t event_type; 
} ext_event_data_t; 

typedef __packed struct 
{ 
    uint8_t supply_projectile_id;   
    uint8_t supply_robot_id;   
    uint8_t supply_projectile_step;   
uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t; 

typedef __packed struct 
{ 
    uint8_t supply_projectile_id;  
    uint8_t supply_robot_id;   
uint8_t supply_num;  
} ext_supply_projectile_booking_t; 

typedef __packed struct 
{ 
    uint8_t level;  
    uint8_t foul_robot_id;   
} ext_referee_warning_t; 

typedef __packed struct 
{ 
    uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t remain_HP; 
    uint16_t max_HP; 
    uint16_t shooter_heat0_cooling_rate; 
    uint16_t shooter_heat0_cooling_limit; 
    uint16_t shooter_heat1_cooling_rate; 
    uint16_t shooter_heat1_cooling_limit; 
    uint8_t mains_power_gimbal_output : 1; 
    uint8_t mains_power_chassis_output : 1; 
    uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_status_t; 

typedef __packed struct 
{ 
    uint16_t chassis_volt;   
    uint16_t chassis_current;   
    float chassis_power;   
    uint16_t chassis_power_buffer;   
    uint16_t shooter_heat0;   
    uint16_t shooter_heat1;   
} ext_power_heat_data_t; 

typedef __packed struct 
{ 
    float x; 
    float y; 
    float z; 
    float yaw; 
} ext_game_robot_pos_t; 
 

 typedef __packed struct 
{ 
    uint8_t power_rune_buff; 
}ext_buff_t; 

typedef __packed struct 
{ 
    uint8_t energy_point; 
    uint8_t attack_time; 
} aerial_robot_energy_t; 

typedef __packed struct 
{ 
    uint8_t armor_id : 4; 
    uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 
 
typedef __packed struct 
{ 
    uint8_t bullet_type;  
    uint8_t bullet_freq;  
    float bullet_speed;  
} ext_shoot_data_t; 

typedef __packed struct 
{ 
    uint16_t bullet_remaining_num;    
} ext_bullet_remaining_t; 

typedef __packed struct 
{ 
    uint16_t data_cmd_id;  
    uint16_t sender_ID;  
    uint16_t receiver_ID; 
}ext_student_interactive_header_data_t; 

#endif // __APP_JUDGE_UART_DATASTRUCT_H
