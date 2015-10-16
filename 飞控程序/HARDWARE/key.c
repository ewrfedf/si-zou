#include "key.h"
#include "SysTick.h"

/*
 * ��������Key_Init
 * ����  ��������ʼ��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//����IO������ �ṹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//ѡ������0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//����GPIOA_0����

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;//ѡ������13 15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//����GPIOA_0����			
}

/*
 * ��������Key_Scan
 * ����  ������ɨ��
 * ����  ����
 * ���  ��
 *		  0:û�а�������
 *		  1:KEY0����
 *		  2:KEY1����
 *		  3:KEY2����
 * ����  ���ⲿ���� 
 */
u8 Key_Scan(void)
{
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1);
			return 3;
		}
		else return 0;
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0) 
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0);	
			return 2;
		}
		else return 0;	
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0)
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0);
			return 1;
		}
		else return 0;	
	}
	else return 0;			
}













