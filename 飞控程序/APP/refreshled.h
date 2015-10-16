#ifndef __REFRESHLED_H__
#define __REFRESHLED_H__

#include "stm32f10x.h"

//0:
//1:
//2:
//3:
extern volatile u8 led_state;//定义led状态


void Led_Refresh_Init(void );//闪烁初始化
void Led_Flash0(void);//LED 1次慢闪+2次快闪
void Led_Flash1(void);//LED 2次快闪
void Led_Flash2(void);//LED 1次慢闪
void Led_Flash3(void);//LED 4次快闪



#endif













