#include "rtc.h"

tm timer;//ʱ�ӽṹ��
u16 CNTL;

//�·�   1  2  3  4  5  6  7  8  9  10 11 12
//����   31 29 31 30 31 30 31 31 30 31 30 31
//������ 31 28 31 30 31 30 31 31 30 31 30 31


/*
 * ��������RTC_NVIC_Config
 * ����  ��RTC�ж����� ��ռ3 ��Ӧ3
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void RTC_NVIC_Config(void)
{	
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;		//RTCȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//ʹ�ܸ�ͨ���ж�
	NVIC_Init(&NVIC_InitStructure);		//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

/*
 * ��������Is_Leap_Year
 * ����  ���ж��Ƿ������꺯��
 * ����  ��year ���
 * ���  ��������ǲ������� 1,�� 0,����
 * ����  ���ڲ�����
 */
u8 Is_Leap_Year(u16 year)//(�ܱ�4����ȴ���ܱ�100����)����(�ܱ�400����)�����
{
	if((year%4==0 && year%100!=0)||(year%400==0))
		return 1;
	else
		return 0;	
}

/*
 * ��������RTC_Set
 * ����  ������ʱ�� ��1970��1��1��Ϊ��׼,1970~2099��Ϊ�Ϸ����
 * ����  ���� �� �� ʱ �� ��
 * ���  ��0,�ɹ�;����:�������
 * ����  ���ⲿ����
 */
u8 const mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};//ƽ����·����ڱ�
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 shour,u8 smin,u8 ssec)
{
	u16 i;
	u32 seccount=0;
	if(syear<1970||syear>2099) return 1;
	for(i=1970;i<syear;i++)
	{
		if(Is_Leap_Year(i)) seccount += 31622400; //�����������
		else seccount += 31536000;  //ƽ���������
	}
	smon--;
	for(i=0;i<smon;i++)
	{
		if(Is_Leap_Year(syear)&&i == 1) seccount += 86400;//����2�·�����һ���������
		seccount += (u32)mon_table[i]*86400;//�·����������		
	}
	seccount += (u32)(sday-1)*86400;//�������������
	seccount += (u32)(shour)*3600;//Сʱ���������	
	seccount += (u32)(smin)*60; //����������
	seccount += ssec;//�������Ӽ���ȥ

	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);//ʹ�ܵ�Դʱ�� ʹ�ܱ���ʱ��
	PWR_BackupAccessCmd(ENABLE);//ʹ��RTC�ͺ󱸼Ĵ�������
	
	RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������	
	RTC_SetCounter(seccount);//����RTC��������ֵ
	RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
	return 0;
	
}

/*
 * ��������RTC_Get
 * ����  ���õ���ǰ��ʱ��
 * ����  ����
 * ���  ��0,�ɹ�;����:�������.
 * ����  ���ⲿ����
 */
u8 RTC_Get(void)
{
	static u16 daycnt=0;//ֻ�ڸ�ֵһ��
	u32 timecount=0;
	u32 temp=0;
	u16 temp1=0;
	
	RTC_WaitForSynchro();//�ȴ�RTC_XXX�Ĵ���ͬ��
	timecount = RTC_GetCounter();//�õ��������е�ֵ(������)
   	
	temp = timecount/86400;	 //�õ�����(��������Ӧ��)
	
	if(daycnt!=temp)//����һ����
	{
		daycnt = temp;
		temp1 = 1970; 	//��1970�꿪ʼ
		while(temp>=365)
		{
			if(Is_Leap_Year(temp1))//������
			{
				if(temp >=366) temp-=366;
				else break;
			}
			else temp -= 365;
			temp1++;
		}
		timer.w_year = temp1;//�õ����

		temp1 = 0;
		while(temp>=28)
		{
			if(Is_Leap_Year(timer.w_year)&&temp1 == 1)//������·�
			{
				if(temp>=29) temp -= 29;
				else break;
			}
			else
			{
				if(temp>=mon_table[temp1]) temp -= mon_table[temp1];//ƽ��
				else break;
			}
			temp1++;
		}
		timer.w_month = temp1+1;//�õ��·�
		timer.w_date = temp+1;//�õ�����
	}
	temp = timecount%86400;//�õ�ʣ��һ��������
	timer.hour = temp/3600;//Сʱ
	timer.min = temp%3600/60;//����
	timer.sec = temp%3600%60;//����
	timer.week=RTC_Get_Week(timer.w_year,timer.w_month,timer.w_date);//��ȡ����  
	return 0;
}



/*
 * ��������RTC_Get_Week
 * ����  ��������������ڼ� 1970~2099��Ϊ�Ϸ���� ���á���ķ����ɭ���㹫ʽ��
 * ����  ���� �� ��
 * ���  �����ں�
 * ����  ���ⲿ����
 */																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{
	int week;
	if (month == 1 || month == 2)
    {
    	month += 12;
        year--;
    }	
	week =(day+2*month+3*(month+1)/5+year+year/4-year/100+year/400)%7;
	return week;
}


/*
 * ��������RTC_Init
 * ����  ����ʼ��RTCʱ��,ͬʱ���ʱ���Ƿ�������
 * ����  ��key ����ʶ��Կ��
 * ���  ��0���� 1�������
 * ����  ���ⲿ����
 */
u8 RTC_Init(u16 key)
{
	u8 temp = 0;
	RTC_NVIC_Config();
	if(BKP_ReadBackupRegister(BKP_DR1)!=key)//��һ������
	{	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);//ʹ�ܵ�Դʱ�� ʹ�ܱ���ʱ��
		PWR_BackupAccessCmd(ENABLE);//ʹ��RTC�ͺ󱸼Ĵ�������
		
		BKP_DeInit();//������BKP��ȫ���Ĵ�������Ϊȱʡֵ
		RCC_LSEConfig(RCC_LSE_ON);	//�����ⲿ���پ���(LSE),ʹ��������پ���

		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) //�ȴ��ⲿʱ�Ӿ���	 
		{
			temp++;
			delay_ms(10);
			if(temp>=250) return 1;
		}

		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//����LSE��ΪRTCʱ��
		RCC_RTCCLKCmd(ENABLE);//����RTCʱ��
		
		//RTC_WaitForSynchro();//�ȴ�RTC_XXX�Ĵ���ͬ��
		RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		RTC_ITConfig(RTC_IT_SEC,ENABLE);//�������ж�
		RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		RTC_SetPrescaler(32767);//����RTCԤ��Ƶ��ֵ
		RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		
		//RTC_Set(2012,12,9,21,47,0);
		Auto_Time_Set();
		BKP_WriteBackupRegister(BKP_DR1,0x6060);//��ָ���ĺ󱸼Ĵ�����д���û���������		
	}
	else
	{
		RTC_ITConfig(RTC_IT_SEC,ENABLE);//�������ж�
		RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������	
	}
	RTC_Get();//����ʱ��
	return 0; //ok				
}



//RTC�жϷ�����//
void RTC_IRQHandler(void)
{
	if(RTC->CRL&(1<<0))//���ж�
	{
		while(!(RTC->CRL&(1<<3)));//�ȴ�RTC_XXX�Ĵ���ͬ��
		CNTL = RTC->CNTL;
		RTC_Get();//����ʱ��		
	}
	if(RTC->CRL&(1<<1))//�����ж�
	{
		/***�����жϴ���***/			
	}
	if(RTC->CRL&(1<<2))//����ж�
	{
		/***����жϴ���***/		
	}
	RTC->CRL &= 0xFFF8;         //��������жϱ��־ [2]��� [1]���� [0]��
	RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
}


/*�����������¡���δŪ��������������*/
//�Ƚ������ַ���ָ�����ȵ������Ƿ����
//����:s1,s2Ҫ�Ƚϵ������ַ���;len,�Ƚϳ���
//����ֵ:1,���;0,�����
u8 str_cmpx(u8*s1,u8*s2,u8 len)
{
	u8 i;
	for(i=0;i<len;i++)if((*s1++)!=*s2++)return 0;
	return 1;	   
}

extern const u8 *COMPILED_DATE;//��ñ�������
extern const u8 *COMPILED_TIME;//��ñ���ʱ��
const u8 Month_Tab[12][3]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"}; 
//�Զ�����ʱ��Ϊ������ʱ��   
void Auto_Time_Set(void)
{
	u8 temp[3];
	u8 i;
	u8 mon,date;
	u16 year;
	u8 sec,min,hour;
	for(i=0;i<3;i++)temp[i]=COMPILED_DATE[i];   
	for(i=0;i<12;i++)if(str_cmpx((u8*)Month_Tab[i],temp,3))break;	
	mon=i+1;//�õ��·�
	if(COMPILED_DATE[4]==' ')date=COMPILED_DATE[5]-'0'; 
	else date=10*(COMPILED_DATE[4]-'0')+COMPILED_DATE[5]-'0';  
	year=1000*(COMPILED_DATE[7]-'0')+100*(COMPILED_DATE[8]-'0')+10*(COMPILED_DATE[9]-'0')+COMPILED_DATE[10]-'0';	   
	hour=10*(COMPILED_TIME[0]-'0')+COMPILED_TIME[1]-'0';  
	min=10*(COMPILED_TIME[3]-'0')+COMPILED_TIME[4]-'0';  
	sec=10*(COMPILED_TIME[6]-'0')+COMPILED_TIME[7]-'0';  
	RTC_Set(year,mon,date,hour,min,sec);
	//printf("%d-%d-%d  %d:%d:%d\n",year,mon,date,hour,min,sec);
} 



