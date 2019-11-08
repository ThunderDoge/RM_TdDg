#ifndef __BSP_Beep_H
#define __BSP_Beep_H

#include "gpio.h"
#include "tim.h"

typedef struct
{
	unsigned short int a; //代表音符
	unsigned char p;      //代表音符节拍
}song;
//歌曲结构体 记得加结束符 格式为{对应音，该音是什么音符}

void Music_Handle(void);//将这个函数放在定时器更新中断里面 建议0.5s为一次周期 可根据歌曲特性改变周期
void Music_Play(song* head);
void Music_Init(void);
void Music_Stop(void); //音乐停止

extern song warning[];
extern song warning1_L[];
extern song warning2_L[];
extern song warning3_L[];
extern song warning4_L[];
extern song warning5_L[];
extern song warning6_L[];
extern song warning7_L[];
extern song warning8_L[];
extern song warning1_M[];
extern song warning2_M[];
extern song warning3_M[];
extern song warning4_M[];
extern song warning5_M[];
extern song warning6_M[];
extern song warning7_M[];
extern song warning8_M[];
extern song warning1_H[];
extern song warning2_H[];
extern song warning3_H[];
extern song warning4_H[];
extern song warning5_H[];
extern song warning6_H[];
extern song warning7_H[];
extern song warning8_H[];
extern song warningx[];
extern song jile[];
extern song Castle_in_the_Sky[];
extern song ItsOK[];
extern song Little_star[];
extern song INTEL[];
extern song Song_of_Joy[];
extern song Kontora[];
extern song News[];

extern unsigned char IsDance;

#define BEEP_ON __HAL_TIM_SET_AUTORELOAD(&MTIM,1000);\
                __HAL_TIM_SET_COMPARE(&MTIM,MCH,501);\
                HAL_TIM_PWM_Start(&MTIM,MCH);
#define BEEP_OFF HAL_TIM_PWM_Stop(&MTIM,MCH);

#endif

