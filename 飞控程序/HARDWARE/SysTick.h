#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "stm32f10x.h"


void Delay_Init(u8 SYSCLK);//��ʱ������ʼ��
void delay_ms(u16 nms);//ms��ʱ����
void delay_us(u32 nus);//us��ʱ����



#endif


