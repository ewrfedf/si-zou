#include "SysTick.h"

static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������


/*
 * ��������Delay_Init
 * ����  ��SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
 * ����  ��SYSCLK(ϵͳʱ��/1MHz)
 * ���  ����
 * ����  ���ⲿ���� 
 */
void Delay_Init(u8 SYSCLK)
{
	//bit2���,ѡ���ⲿʱ��  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}

/*
 * ��������delay_ms
 * ����  ��ms��ʱ����
 * ����  ��nms ���ֵ1864!!!
 * ���  ����
 * ����  ���ⲿ���� 
 */
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
}

/*
 * ��������delay_us
 * ����  ��us��ʱ����
 * ����  ��nus ���ֵ1864135!!! 
 * ���  ����
 * ����  ���ⲿ���� 
 */
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}





