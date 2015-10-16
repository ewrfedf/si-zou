#ifndef __RC_H__
#define __RC_H__

#include "stm32f10x.h"

#define MAX_ANGLE 30
#define MAX_GYRO 280 //���ת�����ٶ�280��ÿ��

extern u16 RC_CH[8];

void RC_Init(void);
void TIM4_Init(void);


u16 Value_2_Thr(void);
float Value_2_Roll(void);
float Value_2_Pitch(void);
s16 Vaule_2_Gyro(void);


void Channel_Adjust(void);//ң����RC��ͨ��У׼(����ң����У׼����У׼��Ϻ���븴λ�����˳���)
u8 Channel_Config(void);//��ȡFlash��ĵ�RCͨ��У׼ֵ



#endif

