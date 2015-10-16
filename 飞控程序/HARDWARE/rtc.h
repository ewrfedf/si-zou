#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f10x.h"
#include "SysTick.h"

//ʱ��ṹ��
typedef struct 
{
	u8 hour;
	u8 min;
	u8 sec;			
	//������������
	u16 w_year;
	u8  w_month;
	u8  w_date;
	u8  week;		 
}tm;					 
extern tm timer; 
extern u8 const mon_table[12];//�·��������ݱ�
extern u16 CNTL;

u8 RTC_Init(u16 key);//��ʼ��RTC,����0,ʧ��;1,�ɹ�;
u8 Is_Leap_Year(u16 year);//ƽ��,�����ж�
u8 RTC_Get(void);//����ʱ��
u8 RTC_Get_Week(u16 year,u8 month,u8 day);
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 shour,u8 smin,u8 ssec);//����ʱ��
void Auto_Time_Set(void);//����ʱ��Ϊ����ʱ��


#endif


