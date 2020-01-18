/* -- Encoding = GB2312 -- */
/**
  * @file  SentryTroubleShooter.cpp
  * @brief    SentryTroubleShooter 哨兵故障自诊断功能，试作型01号
  * @details  使用RGB
  * @author   ThunderDoge
  * @date     2019/12/25
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */
 #include "SentryTroubleShooter.hpp"

uint8_t RGB_LoopScript[30] = {1,2,4,0};
uint16_t RGB_ScriptDuration[30] = {1000,1000,1000,1000};
uint32_t RGB_UsingFrame=4;
uint32_t RGB_FrameTime=200;

void SentryTroubleShooter_Init(void)
{
	RGB_SetByCode(0x00);
}


void RGB_SetByCode(uint8_t rgb_code)
{
    uint8_t r_code = (rgb_code ^ (1U<<0)) != 0;
    uint8_t g_code = (rgb_code ^ (1U<<1)) != 0;
    uint8_t b_code = (rgb_code ^ (1U<<2)) != 0;
    HAL_GPIO_WritePin(RGB1_GPIO_Port,RGB1_Pin,r_code?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB2_GPIO_Port,RGB2_Pin,g_code?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB3_GPIO_Port,RGB3_Pin,b_code?GPIO_PIN_SET:GPIO_PIN_RESET);
}
int i=0;
void CloudRGBAlert_Handle(void)
{
	
	RGB_SetByCode(RGB_LoopScript[i]);
}

void task_SentryTroubleShooter(void* param)
{
    SentryTroubleShooter_Init();
    TickType_t LastTick = xTaskGetTickCount();
    while (1)
    {
        int i;
        for(i=0;i<RGB_UsingFrame;i++)
        {
            RGB_SetByCode (RGB_LoopScript[i]);
            #ifdef CMSIS_OS_H_
			vTaskDelayUntil(&LastTickl,RGB_ScriptDuration[i]);
			#else
			HAL_Delay(RGB_ScriptDuration[i]);
            #endif
        }
    }
}
