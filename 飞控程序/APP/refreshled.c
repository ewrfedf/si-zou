#include "refreshled.h"
#include "led.h"
#include "systime.h"





volatile u8 led_state;//����led״̬

void Led_Refresh_Init(void )//��˸��ʼ��
{
//	TIM1_Config();
	time2_led = 2;
	
	led_state = 1;
}





void Led_Flash0(void) //LED 1������+2�ο���
{
	if(time2_led == 0)
	{
		LED0(ON);
	}
	else if(time2_led == 75)
	{
		LED0(OFF);
	}
		
	if(time2_led == 500)
	{
		LED0(ON);
	}
	else if(time2_led == 525)
	{
		LED0(OFF);
	}
	else if(time2_led == 550)
	{
		LED0(ON);
	}
	else if(time2_led == 575)
	{
		LED0(OFF);
	}
	
}

void Led_Flash1(void) //LED 2�ο���
{
	if(time2_led == 500)
	{
		LED0(ON);
	}
	else if(time2_led == 525)
	{
		LED0(OFF);
	}
	else if(time2_led == 550)
	{
		LED0(ON);
	}
	else if(time2_led == 575)
	{
		LED0(OFF);
	}

}

void Led_Flash2(void) //LED 1������
{
	if(time2_led == 0)
	{
		LED0(ON);
	}
	else if(time2_led == 75)
	{
		LED0(OFF);
	}
}


void Led_Flash3(void) //LED 4�ο���
{
	if(time2_led == 500)
	{
		LED0(ON);
	}
	else if(time2_led == 525)
	{
		LED0(OFF);
	}
	else if(time2_led == 550)
	{
		LED0(ON);
	}
	else if(time2_led == 575)
	{
		LED0(OFF);
	}
	else if(time2_led == 600)
	{
		LED0(ON);
	}
	else if(time2_led == 625)
	{
		LED0(OFF);
	}
	else if(time2_led == 650)
	{
		LED0(ON);
	}
	else if(time2_led == 675)
	{
		LED0(OFF);
	}
}











