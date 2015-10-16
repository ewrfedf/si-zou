#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x.h"

#define ON 0
#define OFF 1

#define LED0(a) if(a) \
					GPIO_SetBits(GPIOC,GPIO_Pin_11); \
				else \
					GPIO_ResetBits(GPIOC,GPIO_Pin_11);
#define LED1(a) if(a) \
					GPIO_SetBits(GPIOC,GPIO_Pin_10); \
				else \
					GPIO_ResetBits(GPIOC,GPIO_Pin_10);
				

void LED_GPIO_Config(void);//≈‰÷√LEDµΩIOø⁄


#endif


