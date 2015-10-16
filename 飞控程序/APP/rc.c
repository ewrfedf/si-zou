#include "rc.h"
#include "Systick.h"
#include "led.h"
#include "maincom.h"
#include "stmflash.h"

//TIM4 PWM  ����ͨ����
//channel1  channel2  channel3  channel4
//PB6       PB7       PB8       PB9

//TIM2 PWM  ����ͨ����
//channel5  channel6
//PA0       PA1     


//��ʱ��4ͨ�����벶������
u8  TIM4CH1_CAPTURE_STA=0;	//ͨ��1���벶��״̬		  �ø���λ�������־������λ���������  				
u16	TIM4CH1_CAPTURE_UPVAL;	//ͨ��1���벶��ֵ
u16	TIM4CH1_CAPTURE_DOWNVAL;//ͨ��1���벶��ֵ

u8  TIM4CH2_CAPTURE_STA=0;	//ͨ��2���벶��״̬		    				
u16	TIM4CH2_CAPTURE_UPVAL;	//ͨ��2���벶��ֵ
u16	TIM4CH2_CAPTURE_DOWNVAL;//ͨ��2���벶��ֵ

u8  TIM4CH3_CAPTURE_STA=0;	//ͨ��3���벶��״̬		    				
u16	TIM4CH3_CAPTURE_UPVAL;	//ͨ��3���벶��ֵ
u16	TIM4CH3_CAPTURE_DOWNVAL;//ͨ��3���벶��ֵ

u8  TIM4CH4_CAPTURE_STA=0;	//ͨ��4���벶��״̬		    				
u16	TIM4CH4_CAPTURE_UPVAL;	//ͨ��4���벶��ֵ
u16	TIM4CH4_CAPTURE_DOWNVAL;//ͨ��4���벶��ֵ

u16 RC_CH[8];

u16 CH1_MxMi[2] = {1500,1500};
u16 CH2_MxMi[2] = {1500,1500};
u16 CH3_MxMi[2] = {1500,1500};
u16 CH4_MxMi[2] = {1500,1500};
	
u16 CH5_Mode[6] = {0,0,0,0,0,0};
u16 CH6_MxMi[2] = {1500,1500};
u16 CH7_MxMi[2] = {1500,1500};
u16 CH8_MxMi[2] = {1500,1500};



void RC_Init(void)
{
	TIM4_Init();
}


void TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM4_ICInitStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//����NVIC��TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//ͨ������ΪTIM4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//���ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//д������
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ��TIM4ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//GPIOB ʱ�ӿ���
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB 6 7 8 9 ������
	GPIO_ResetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	
	TIM_TimeBaseStructure.TIM_Period = 20000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);//ʱ��Ԥ��Ƶ�� 72M/72(1M/20000) = 50HZ
 	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ(0->?)
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//����TIM4
	
	//��ʼ��TIM4���벶�����
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);//����CC1IE�����ж�	 
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);//����CC2IE�����ж�	
	TIM_ITConfig(TIM4,TIM_IT_CC3,ENABLE);//����CC3IE�����ж�	
	TIM_ITConfig(TIM4,TIM_IT_CC4,ENABLE);//����CC4IE�����ж�
	
	TIM_Cmd(TIM4,ENABLE );//ʹ�ܶ�ʱ��4
}

//��ʱ��4�жϷ������	 
void TIM4_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)//����1���������¼�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);//�������1��־λ
		
		if(TIM4CH1_CAPTURE_STA == 0)//����������
		{
			TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);//��ȡ�����ص�����
			
			TIM4CH1_CAPTURE_STA = 1;		//����Բ�����������
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);//����Ϊ�½��ز���
		}
		else                        //�����½��� (�Ѿ�����һ�������ĸߵ�ƽ���壡)
		{
			TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//��ȡ�½��ص�����
			
			//�ж��Ƿ񳬳����������,����ߵ�ƽ����ʱ��us
			if(TIM4CH1_CAPTURE_DOWNVAL<TIM4CH1_CAPTURE_UPVAL)
			{
				RC_CH[0] = 20000 - TIM4CH1_CAPTURE_UPVAL + TIM4CH1_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[0] = TIM4CH1_CAPTURE_DOWNVAL- TIM4CH1_CAPTURE_UPVAL;
			}
			
			TIM4CH1_CAPTURE_STA = 0;
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //����Ϊ�����ز���
		}
	}	


	if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)//����2���������¼�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);//�������2��־λ
		
		if(TIM4CH2_CAPTURE_STA == 0)//����������
		{
			TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);//��ȡ�����ص�����
			
			TIM4CH2_CAPTURE_STA = 1;		//����Բ�����������
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);//����Ϊ�½��ز���
		}
		else                        //�����½��� (�Ѿ�����һ�������ĸߵ�ƽ���壡)
		{
			TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//��ȡ�½��ص�����
			
			//�ж��Ƿ񳬳����������,����ߵ�ƽ����ʱ��us
			if(TIM4CH2_CAPTURE_DOWNVAL<TIM4CH2_CAPTURE_UPVAL)
			{
				RC_CH[1] = 20000 - TIM4CH2_CAPTURE_UPVAL + TIM4CH2_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[1] = TIM4CH2_CAPTURE_DOWNVAL- TIM4CH2_CAPTURE_UPVAL;
			}
			TIM4CH2_CAPTURE_STA = 0;
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //����Ϊ�����ز���
		}
	}
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET)//����3���������¼�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);//�������3��־λ
		
		if(TIM4CH3_CAPTURE_STA == 0)//����������
		{
			TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);//��ȡ�����ص�����
			
			TIM4CH3_CAPTURE_STA = 1;		//����Բ�����������
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);//����Ϊ�½��ز���
		}
		else                        //�����½��� (�Ѿ�����һ�������ĸߵ�ƽ���壡)
		{
			TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//��ȡ�½��ص�����
			
			//�ж��Ƿ񳬳����������,����ߵ�ƽ����ʱ��us
			if(TIM4CH3_CAPTURE_DOWNVAL<TIM4CH3_CAPTURE_UPVAL)
			{
				RC_CH[2] = 20000 - TIM4CH3_CAPTURE_UPVAL + TIM4CH3_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[2] = TIM4CH3_CAPTURE_DOWNVAL- TIM4CH3_CAPTURE_UPVAL;
			}
			
			TIM4CH3_CAPTURE_STA = 0;
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //����Ϊ�����ز���
			
		}
	}
	
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET)//����4���������¼�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);//�������4��־λ
		
		if(TIM4CH4_CAPTURE_STA == 0)//����������
		{
			TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);//��ȡ�����ص�����
			
			TIM4CH4_CAPTURE_STA = 1;		//����Բ�����������
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);//����Ϊ�½��ز���
		}
		else                        //�����½��� (�Ѿ�����һ�������ĸߵ�ƽ���壡)
		{
			TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//��ȡ�½��ص�����
			
			//�ж��Ƿ񳬳����������,����ߵ�ƽ����ʱ��us
			if(TIM4CH4_CAPTURE_DOWNVAL<TIM4CH4_CAPTURE_UPVAL)
			{
				RC_CH[3] = 20000 - TIM4CH4_CAPTURE_UPVAL + TIM4CH4_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[3] = TIM4CH4_CAPTURE_DOWNVAL- TIM4CH4_CAPTURE_UPVAL;
			}
			
			TIM4CH4_CAPTURE_STA = 0;
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //����Ϊ�����ز���
			
		}
	}
	
}
//////////////////////////////////////////////////////////RC���幦��ʵ��///////////////////////////////////////////////////////////


u16 Get_RightCH_Value(u16 num,u16 max,u16 min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}



//��CH3ͨ������������ź�ת����У׼�������ֵ
u16 Value_2_Thr(void)
{
	return (1000+RC_CH[3-1]-CH3_MxMi[1]);
}

//��CH1ͨ����PWMֵת����������Roll�Ƕ�
float Value_2_Roll(void)
{
	s16 value_half;
	
	value_half = (CH3_MxMi[0]+CH3_MxMi[1])/2;
	
	return -(float)MAX_ANGLE*(float)((s16)RC_CH[1-1]-value_half)/(float)(value_half-CH3_MxMi[0]);
	
}


//��CH2ͨ����PWMֵת����������Pitch�Ƕ�
float Value_2_Pitch(void)
{
	s16 value_half;
	
	value_half = (CH2_MxMi[0]+CH2_MxMi[1])/2;//�����е�ֵ
	
	return (float)MAX_ANGLE*(float)((s16)RC_CH[2-1]-value_half)/(float)(value_half-CH2_MxMi[0]);
	
}

//��CH4ͨ����PWMֵת����������Yawת�����ٶ�
s16 Vaule_2_Gyro(void)
{
	s16 value_half;
	
	value_half = (CH4_MxMi[0]+CH4_MxMi[1])/2;//�����е�ֵ
	
	return MAX_GYRO*((s16)RC_CH[4-1]-value_half)/(value_half-CH4_MxMi[0]);
	
}


//ң����RC��ͨ��У׼(����ң����У׼����У׼��Ϻ���븴λ�����˳���)
void Channel_Adjust(void)
{
	u16 flash_data[39];
	u16 i = 0;
	u8 point;
	LED1(0);
	for(i=0;i<1000;i++)//20s�����ң����У׼
	{
		if(RC_CH[1-1]>CH1_MxMi[0] && RC_CH[1-1]<=2500) CH1_MxMi[0] = RC_CH[1-1];//�õ����ֵ
		if(RC_CH[1-1]<CH1_MxMi[1] && RC_CH[1-1]>=500) CH1_MxMi[1] = RC_CH[1-1];//�õ���Сֵ
	
		if(RC_CH[2-1]>CH2_MxMi[0] && RC_CH[2-1]<=2500) CH2_MxMi[0] = RC_CH[2-1];//�õ����ֵ
		if(RC_CH[2-1]<CH2_MxMi[1] && RC_CH[2-1]>=500) CH2_MxMi[1] = RC_CH[2-1];//�õ���Сֵ
	
		if(RC_CH[3-1]>CH3_MxMi[0] && RC_CH[3-1]<=2500) CH3_MxMi[0] = RC_CH[3-1];//�õ����ֵ
		if(RC_CH[3-1]<CH3_MxMi[1] && RC_CH[3-1]>=500) CH3_MxMi[1] = RC_CH[3-1];//�õ���Сֵ
	
		if(RC_CH[4-1]>CH4_MxMi[0] && RC_CH[4-1]<=2500) CH4_MxMi[0] = RC_CH[4-1];//�õ����ֵ
		if(RC_CH[4-1]<CH4_MxMi[1] && RC_CH[4-1]>=500) CH4_MxMi[1] = RC_CH[4-1];//�õ���Сֵ
		
		LED0(ON);
		delay_ms(20);
		LED0(OFF);
	}
	
	for(i=0;i<3;i++)//LED����˸3�±�ʾУ׼���
	{
		LED0(ON);
		delay_ms(100);
		LED0(OFF);
		delay_ms(900);
	}
	
	STMFLASH_Read(PAGE1,flash_data,39);//��flash��������ȶ�����
	
	//�ٸ�ֵ
	point = 0;
	
	flash_data[point++] = ADJUST_FLAG;//д��У׼���λ
	//1-4ͨ����ֵ
	flash_data[point++] = CH1_MxMi[0];  flash_data[point++] = CH1_MxMi[1];
	flash_data[point++] = CH2_MxMi[0];  flash_data[point++] = CH2_MxMi[1];
	flash_data[point++] = CH3_MxMi[0];  flash_data[point++] = CH3_MxMi[1];
	flash_data[point++] = CH4_MxMi[0];  flash_data[point++] = CH4_MxMi[1];
	
	//5-8ͨ����ֵ
	//???????????????????????????
	
	STMFLASH_Write(PAGE1,flash_data,39);//������д��flash
	
	//����У׼���� �� ���͵���λ��
	STMFLASH_Read(PAGE1,flash_data,39);
	
	printf("CH1 = %d,%d\n",flash_data[1],flash_data[2]);
	printf("CH2 = %d,%d\n",flash_data[3],flash_data[4]);
	printf("CH3 = %d,%d\n",flash_data[5],flash_data[6]);
	printf("CH4 = %d,%d\n",flash_data[7],flash_data[8]);
	
	LED0(ON);
	while(1);
}


//��ȡFlash��ĵ�RCͨ��У׼ֵ
u8 Channel_Config(void)
{
	u16 flash_data[39];
	u8 point=0;
	
	//����Flash����
	STMFLASH_Read(PAGE1,flash_data,29);
	
	if(flash_data[point++] == ADJUST_FLAG)//�ж��Ƿ��Ѿ�����ң����У׼
	{
		CH1_MxMi[0] = flash_data[point++];CH1_MxMi[1] = flash_data[point++];
		CH2_MxMi[0] = flash_data[point++];CH2_MxMi[1] = flash_data[point++];
		CH3_MxMi[0] = flash_data[point++];CH3_MxMi[1] = flash_data[point++];
		CH4_MxMi[0] = flash_data[point++];CH4_MxMi[1] = flash_data[point++];
		
		return 1;
	}
	else
	{
		CH1_MxMi[0] = 2000;CH1_MxMi[1] = 1000;
		CH2_MxMi[0] = 2000;CH2_MxMi[1] = 1000;
		CH3_MxMi[0] = 2000;CH3_MxMi[1] = 1000;
		CH4_MxMi[0] = 2000;CH4_MxMi[1] = 1000;
		
		return 0;
	}
}

