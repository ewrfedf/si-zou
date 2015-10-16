#include "maincom.h"
#include "dma.h"
#include "control.h"

u8 DMA_DATA_SEND_FLAG;
u8 Receive_Complete;

/*
 * ��������USART1_Config
 * ����  ��USART1 GPIO ����,����ģʽ���á�115200 8-N-1
 *		 : �����ж�ģʽ
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void USART1_Config(u32 speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//����GPIOA��USART1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

	//IO������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//PA9�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA10�������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//IO������д��

	//USART1����
	USART_InitStructure.USART_BaudRate = speed;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//����ֹͣλ 1λ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//���շ���ʹ��
	
	USART_Init(USART1, &USART_InitStructure);//USART1����д��
	USART_Cmd(USART1, ENABLE);//USART1ʹ��
	
	//USART1�����ж�ʹ��
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	//����NVIC��USART1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//ͨ������ΪUSART1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//��ռ3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//��USART1�ж�1ͨ��
	NVIC_Init(&NVIC_InitStructure);//д������ 
 			
}

/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
int fputc(int ch, FILE *f)
{
	/* ��Printf���ݷ������� */
	USART_SendData(USART1, (unsigned char) ch);//USART1�������ݺ���
  	while (!(USART1->SR & USART_FLAG_TXE));//�ж��Ƿ�����"������λ"�Ĵ���
	return (ch);	
}


//USART1 ����һ���ֽ�
void USART1_Send1Char(u8 c)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	USART_SendData(USART1,c);
}

u8 recv_dat[32];
u8 recv_point=0,recv_buf=0;
//*****USART1�жϺ���*****//
void USART1_IRQHandler(void)
{
	u8 temp;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)//�����ж�
	{ 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����ж�λ
		
		temp = USART_ReceiveData(USART1);
		
		if(temp == 0x8A)
		{
			recv_buf = 1;//��ʼ��������
		}
		
		if(recv_buf == 1)
		{
			recv_dat[recv_point++] = temp;
			
			if(recv_point == 32)
			{
				recv_point = 0;
				recv_buf = 0;//�Ѿ�������32�����ݣ�ֹͣ����
				Receive_Complete = 1;//��־�Ѿ����յ���32λ����
			}
		}
	} 	
}


/////////////////////////////////////////////////////���ڷ���/////////////////////////////////////////////////////////////////////


//������̬�Ǻʹ���������
void Data_Send_Attitude(s16* acc,s16* gyro,s16* mag,float rool,float pitch,float yaw)
{
	u8 updata[32];
	u8 sum,i,point=0;
	s16 angle[3];
	
	angle[0] = (s16)(rool*100);
	angle[1] = (s16)(pitch*100);
	angle[2] = (s16)(yaw*10);
	
	updata[point++] = 0x88;
	updata[point++] = 0xAF;
	updata[point++] = 0x1C;
	
	for(i=0;i<3;i++)
	{
		updata[point++] = acc[i]>>8;
		updata[point++] = acc[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = gyro[i]>>8;
		updata[point++] = gyro[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = mag[i]>>8;
		updata[point++] = mag[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = angle[i]>>8;
		updata[point++] = angle[i]&0xFF;
	}
	for(i=0;i<4;i++)
	{
		updata[point++] = 0x00;
	}
	
	for(i=0;i<31;i++)//У���
	{
		sum += updata[i];
	}
	updata[31] = sum;
	
	DMA1_USART1_SEND((u32)updata,32);	
}

//����ң������ �� ���PWM����
void Data_Send_Control(u16 *rc_ch,u16 PWM1,u16 PWM2,u16 PWM3,u16 PWM4,u16 votage)
{
	u8 updata[32];
	u8 i,sum,point=0;
	
	updata[point++] = 0x88;
	updata[point++] = 0xAE;
	updata[point++] = 0x12;
	
	updata[point++] = rc_ch[3-1]>>8;//3ͨ�� ����
	updata[point++] = rc_ch[3-1]&0xFF;
	
	updata[point++] = rc_ch[4-1]>>8;//4ͨ�� ƫ��
	updata[point++] = rc_ch[4-1]&0xFF;
	
	updata[point++] = rc_ch[1-1]>>8;//1ͨ�� ���
	updata[point++] = rc_ch[1-1]&0xFF;
	
	updata[point++] = rc_ch[2-1]>>8;//2ͨ�� ����
	updata[point++] = rc_ch[2-1]&0xFF;
	
	for(i=0;i<5;i++)//ң������ֵ
	{
		updata[point++] = 0;//��8λ
		updata[point++] = 0;//��8λ
	}
	
	updata[point++] = PWM1>>8;
	updata[point++] = PWM1&0xFF;
	updata[point++] = PWM2>>8;
	updata[point++] = PWM2&0xFF;
	updata[point++] = PWM3>>8;
	updata[point++] = PWM3&0xFF;
	updata[point++] = PWM4>>8;
	updata[point++] = PWM4&0xFF;
	
	
	
	updata[point++] = votage>>8;
	updata[point++] = votage&0xFF;
	
	for(i=0;i<31;i++)//У���
	{
		sum += updata[i];
	}
	updata[31] = sum;
	
	
	DMA1_USART1_SEND((u32)updata,32);
}

//����ƫ������
void Data_Send_Offset(s16 acc_offset_x,s16 acc_offset_y,s16 acc_offset_z,s16 gyro_offset_x,s16 gyro_offset_y,s16 gyro_offset_z)
{
	u8 updata[32];
	u8 i,sum,point=0;
	
	updata[point++]=0x88;
	updata[point++]=0xAC;
	updata[point++]=0x1C;
	updata[point++]=0xAC;
	
	updata[point++]=acc_offset_x>>8;
	updata[point++]=acc_offset_x&0xFF;
	
	updata[point++]=acc_offset_y>>8;
	updata[point++]=acc_offset_y&0xFF;
	
	updata[point++]=acc_offset_z>>8;
	updata[point++]=acc_offset_z&0xFF;
	
	updata[point++]=gyro_offset_x>>8;
	updata[point++]=gyro_offset_x&0xFF;
	
	updata[point++]=gyro_offset_y>>8;
	updata[point++]=gyro_offset_y&0xFF;
	
	updata[point++]=gyro_offset_z>>8;
	updata[point++]=gyro_offset_z&0xFF;
	
	sum = 0;
	for(i=0;i<31;i++)
	{
		sum += updata[i];
	}
	
	updata[31]=sum;
	
	//NRF_TxPacket(NRF24L01_TXDATA,32);
	
	DMA1_USART1_SEND((u32)updata,32);
}

//����PID����
void Data_Send_PID(float Roll_P,float Roll_I,float Roll_D,
									 float Pitch_P,float Pitch_I,float Pitch_D,
								   float Yaw_P,float Yaw_I,float Yaw_D)
{
	u8 updata[32];
	u8 i,sum,point=0;
	u16 temp;
	
	updata[point++]=0x88;
	updata[point++]=0xAC;
	updata[point++]=0x1C;
	updata[point++]=0xAD;
	
	temp = Roll_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Roll_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Roll_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	temp = Pitch_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Pitch_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Pitch_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	temp = Yaw_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Yaw_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Yaw_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	sum = 0;
	for(i=0;i<31;i++)
	{
		sum += updata[i];
	}
	
	updata[31]=sum;
	
	DMA1_USART1_SEND((u32)updata,32);
}


//////////////////////////////////////////////////////////////���ڽ���/////////////////////////////////////////////////////////////

//�жϴ���λ�����յ�������
//����ֵ��
//0:��Ч����
//1:У׼ң����
//2:��ȡPIDֵ
//3:��λ�����ɿط���PID����
u8 Recv_Command(void)
{
	
	if(Receive_Complete == 1)//���յ�����
	{
		Receive_Complete = 0;
		
		if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //У׼ң����
				&& recv_dat[3] == 0xAA && recv_dat[4] == 0xA3)
		{
			return 1;
		}
		else if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //��ȡPIDֵ
				&& recv_dat[3] == 0xAD)
		{
			return 2;
		}
		else if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //��λ�����ɿط���PID����
				&& recv_dat[3] == 0xAE)
		{
			return 3;
		}
		else return 0;
	}
	else return 0;//û�н��յ�����
}

//����λ����ȡPIDֵ
void Get_Recv_PID(void)
{
	u8 point = 4;
	u16 temp;
	
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_ROL.P = (float)temp/1000;	
	temp = recv_dat[point++];//��8λMMM
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_ROL.I = (float)temp/1000;
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_ROL.D = (float)temp/1000;
	
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_PIT.P = (float)temp/1000;
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_PIT.I = (float)temp/1000;
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_PIT.D = (float)temp/1000;
	
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_YAW.P = (float)temp/1000;
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_YAW.I = (float)temp/1000;
	temp = recv_dat[point++];//��8λ
	temp <<= 8;
	temp |= recv_dat[point++];//��8λ
	PID_YAW.D = (float)temp/1000;
	
}


