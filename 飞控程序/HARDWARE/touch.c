#include "touch.h"

Pen_Holder Pen_Point;//�����ʵ�� //Ĭ��Ϊtouchtype=0������.

u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;

//SPIд����
//д��1byte����
void ADS_Write_Byte(u8 num)
{
	u8 count;
	TCLK_L;
	for(count=0;count<8;count++)
	{
		if(num&0x80) TDIN_H;
		else TDIN_L;
		num <<= 1;
		TCLK_L;
		TCLK_H;//��������Ч 
	}	
}

//SPI������ 
//��ȡadcֵ
u16 ADS_Read_AD(u8 CMD)	
{
	u8 count;
	u16 Num=0;
	TCLK_L;//����ʱ�� 
	TCS_L;//ѡ��XXXXX
	ADS_Write_Byte(CMD);//����������
	delay_us(6);//XXXXXת��ʱ���Ϊ6us
	TCLK_H;//��1��ʱ�ӣ����BUSY   	    
	TCLK_L;
	for(count=0;count<15;count++)
	{
		Num <<= 1;
		TCLK_H;  	    
		TCLK_L;
		if(DOUT_read) Num++;	
	}
	Num >>= 3;//��12λ��Ч
	TCS_H;//�ͷ�XXXXX
	return(Num);	
}

//��ȡһ������ֵ
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
#define READ_TIMES 15 //��ȡ����
#define LOST_VAL 5	  //����ֵ

u16 ADS_Read_XY(u8 xy,u8 delay)
{
	u16 i,j;
	u16 buf[READ_TIMES];
	u16 sum;
	u16 temp;
	if(delay!=0)
	{
		delay_ms(delay);
	}
	for(i=0;i<READ_TIMES;i++)
	{
		buf[i] = ADS_Read_AD(xy);
	}
	//ð������
	for(i=0;i<READ_TIMES-1;i++)
	{
		for(j=0;j<READ_TIMES-1-i;j++)
		{
			if(buf[j]>buf[j+1])
			{
				temp = buf[j+1];
				buf[j+1] = buf[j];
				buf[j] = temp;	
			}
		}
	}
	////////////
	

	sum = 0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
	{
		sum += buf[i];
	}
	temp = sum/(READ_TIMES-LOST_VAL*2);
	
	return (temp);
}

//���˲��������ȡ
//��Сֵ��������100.
u8 Read_ADS(u16* x,u16* y,u8 delay)
{	
	u16 xtemp,ytemp;
	xtemp = ADS_Read_XY(CMD_RDX,delay);
	ytemp = ADS_Read_XY(CMD_RDY,delay);
	if(xtemp<100||ytemp<100) return 0;//����ʧ��
	*x = xtemp;
	*y = ytemp;
	return 1;//�����ɹ�
}

//2�ζ�ȡADS7846,������ȡ2����Ч��ADֵ,�������ε�ƫ��ܳ���
//50,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
#define ERR_RANGE 50 //��Χ 
u8 Read_ADS2(u16 *x,u16 *y,u8 delay)
{
	u16 x1,x2,y1,y2;
	if(Read_ADS(&x1,&y1,delay)==0) return 0;
	if(Read_ADS(&x2,&y2,delay)==0) return 0;
	if(((s16)x1-(s16)x2<=50&&(s16)x1-(s16)x2>=-50)&&((s16)y1-(s16)y2<=50&&(s16)y1-(s16)y2>=-50))
	{
		*x = (x1+x2)/2;
		*y = (y1+y2)/2;	
		return 1;
	}
	else return 0;	
}

//��ȡһ������ֵ	
//������ȡһ��,ֱ��PEN�ɿ��ŷ���!
u8 Read_TP_Once(void)
{
	u8 t = 0;
	Pen_Int_Set(0);//�ر��ж�
	Pen_Point.Key_Sta=Key_Up;
	Read_ADS2(&Pen_Point.X,&Pen_Point.Y,5);
	while(PEN_read==0&&t<=250)
	{
		t++;
		delay_ms(10);
	}
	Pen_Int_Set(1);//�����ж�
	if(t>250) return 0;//����2.5s ��Ϊ��Ч
	else return 1;
}


//////////////////////////////////////////////
//��LCD�����йصĺ���

//��һ��������
//����У׼�õ�
void Drow_Touch_Point(u8 x,u16 y)
{
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	Draw_Circle(x,y,6);//������Ȧ	
}

//��һ�����
//2*2�ĵ�			   
void Draw_Big_Point(u8 x,u16 y)
{	    
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}

void ADJ_INFO_SHOW(u8*str)
{
	LCD_ShowString(40,40,"x1:       y1:       ",0);
	LCD_ShowString(40,60,"x2:       y2:       ",0);
	LCD_ShowString(40,80,"x3:       y3:       ",0);
	LCD_ShowString(40,100,"x4:       y4:       ",0);
 	LCD_ShowString(40,100,"x4:       y4:       ",0);
 	LCD_ShowString(40,120,str,0);					   
}
///////////////////////////////////////////////////////////


//ת�����
//���ݴ�������У׼����������ת����Ľ��,������X0,Y0��
void Convert_Pos(void)
{
	if(Read_ADS2(&Pen_Point.X,&Pen_Point.Y,0))
	{
		Pen_Point.X0 = Pen_Point.xfac*Pen_Point.X + Pen_Point.xoff;
		Pen_Point.Y0 = Pen_Point.yfac*Pen_Point.Y + Pen_Point.yoff;	
	}	
}

//.�ж�,��⵽PEN�ŵ�һ���½���
//��λPen_Point.Key_StaΪ����״̬
//�ж���0���ϵ��жϼ��
void EXTI1_IRQHandler(void)
{
	Pen_Point.Key_Sta = Key_Down;//��������
	EXTI_ClearITPendingBit(EXTI_Line1);//���LINE1�ϵ��жϱ�־λ  	
}

//PEN�ж�����
void Pen_Int_Set(u8 en)
{ 
	EXTI_InitTypeDef EXTI_InitStructure;
	if(en)//����line1�ϵ��ж�
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//�ⲿ��·EXIT1 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//���� EXTI��·Ϊ�ж����� 
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//����������·�½���Ϊ�ж�����  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ���ⲿ�ж���״̬
		EXTI_Init(&EXTI_InitStructure);//�����ⲿ�ж�
	}
	else//�ر�line1�ϵ��ж�
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//�ⲿ��·EXIT1 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//���� EXTI��·Ϊ�ж����� 
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//����������·�½���Ϊ�ж�����  
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;//�ر��ⲿ�ж���״̬
		EXTI_Init(&EXTI_InitStructure);//�����ⲿ�ж�	
	}	   
}


////////////////////////////////////////////////////////////////////////////////////
//�˲����漰��ʹ���ⲿEEPROM,���û���ⲿEEPROM,���δ˲��ּ���
#ifdef ADJ_SAVE_ENABLE
//������EEPROM����ĵ�ַ�����ַ,ռ��13���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE 40

//����У׼����
void Save_Adjdata(void)
{
	s32 temp;
	temp = Pen_Point.xfac*100000000;//����xУ������ 
	AT24C02_WriteLenByte(SAVE_ADDR_BASE,temp,4);
	temp = Pen_Point.yfac*100000000;//����yУ������
	AT24C02_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);

	AT24C02_WriteLenByte(SAVE_ADDR_BASE+8,Pen_Point.xoff,2);//����xƫ����
	AT24C02_WriteLenByte(SAVE_ADDR_BASE+10,Pen_Point.yoff,2);//����yƫ����
	AT24C02_WriteOneByte(SAVE_ADDR_BASE+12,Pen_Point.touchtype);//���津������
	
	temp=0xFF;//���У׼����
	AT24C02_WriteOneByte(SAVE_ADDR_BASE+13,temp); 	
}

//�õ�������EEPROM�����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
u8 Get_Adjdata(void)
{
	s32 tempfac;
	tempfac = AT24C02_ReadOneByte(SAVE_ADDR_BASE+13);//��ȡ�����,���Ƿ�У׼����
	if(tempfac==0xFF)
	{
		tempfac = AT24C02_ReadLenByte(SAVE_ADDR_BASE,4);//ȡ��xУ������ 
		Pen_Point.xfac = (float)tempfac/100000000;
		tempfac = AT24C02_ReadLenByte(SAVE_ADDR_BASE+4,4);//ȡ��yУ������
		Pen_Point.yfac = (float)tempfac/100000000;
		
		Pen_Point.xoff = (s16)AT24C02_ReadLenByte(SAVE_ADDR_BASE+8,2);//�õ�xƫ����
		Pen_Point.yoff = (s16)AT24C02_ReadLenByte(SAVE_ADDR_BASE+10,2);//�õ�yƫ����
		Pen_Point.touchtype	= AT24C02_ReadOneByte(SAVE_ADDR_BASE+12);//��ȡ�������ͱ��
		if(Pen_Point.touchtype)
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	
		}
		else
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;		
		}
		return 1;	
	}
	else return 0;	
}

#endif
////////////////////////////////////////////////////////////////////////////////


//������У׼����
//�õ��ĸ�У׼����
void Touch_Adjust(void)
{								 
	u16 XY[4][2];
	u8 i=0;
	u16 d1,d2;
	u32 temp1,temp2;
	float fac;
	float xfac_1,xfac_2,yfac_1,yfac_2;
	s16	xoff_1,xoff_2,yoff_1,yoff_2;

	Drow_Touch_Point(20,20);//����1
	while(1)
	{
		
		if(Pen_Point.Key_Sta == Key_Down)
		{
			if(Read_TP_Once())
			{
				XY[i][0] = Pen_Point.X;
				XY[i][1] = Pen_Point.Y;
				LCD_Clear(BLACK);//����
				i++; 
			}
			
			switch(i)
			{
				
				case 1:
				{
					Drow_Touch_Point(220,20);//����1
					break;
				}
				case 2:
				{
					Drow_Touch_Point(20,300);//����1
					break;
				}
				case 3:
				{
					Drow_Touch_Point(220,300);//����1
					break;
				}
				default: break;
			}
		}
		if(i == 4) //ȫ���ĸ����Ѿ��õ�
		{	
			i=0;

			temp1 = fabsf(XY[0][0]-XY[1][0]);//x1-x2
			temp2 = fabsf(XY[0][1]-XY[1][1]);//y1-y2
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//�õ�1,2�ľ���

			temp1 = fabsf(XY[2][0]-XY[3][0]);//x3-x4
			temp2 = fabsf(XY[2][1]-XY[3][1]);//y3-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//�õ�3,4�ľ���

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�����
			{
				Drow_Touch_Point(20,20);//����1
				LCD_ShowNum(40,140,fac*100,3,16,0);//����ֵ������95~105��Χ֮��.
				continue;//��������ѭ��		
			}

			temp1 = fabsf(XY[0][0]-XY[2][0]);//x1-x3
			temp2 = fabsf(XY[0][1]-XY[2][1]);//y1-y3
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//�õ�1,3�ľ���

			temp1 = fabsf(XY[1][0]-XY[3][0]);//x2-x4
			temp2 = fabsf(XY[1][1]-XY[3][1]);//y2-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//�õ�2,4�ľ���

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)
			{
				Drow_Touch_Point(20,20);//����1
				LCD_ShowNum(40,140,fac*100,3,16,0);//����ֵ������95~105��Χ֮��.
				continue;//��������ѭ��		
			}

			temp1 = fabsf(XY[0][0]-XY[3][0]);//x1-x4
			temp2 = fabsf(XY[0][1]-XY[3][1]);//y1-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//�õ�1,4�ľ���

			temp1 = fabsf(XY[1][0]-XY[2][0]);//x2-x3
			temp2 = fabsf(XY[1][1]-XY[2][1]);//y2-y3
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//�õ�2,3�ľ���

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)
			{
				Drow_Touch_Point(20,20);//����1
				LCD_ShowNum(40,140,fac*100,3,16,0);//����ֵ������95~105��Χ֮��.
				continue;//��������ѭ��		
			}

			/*******�������е����� ˵���ϸ�*********/
			xfac_1 = (220-20)/(float)((s16)XY[3][0]-(s16)XY[0][0]);
			xoff_1 = (20*(s16)XY[3][0]-220*(s16)XY[0][0])/((s16)XY[3][0]-(s16)XY[0][0]);
			yfac_1 = (300-20)/(float)((s16)XY[3][1]-(s16)XY[0][1]);
			yoff_1 = (20*(s16)XY[3][1]-300*(s16)XY[0][1])/((s16)XY[3][1]-(s16)XY[0][1]);

			xfac_2 = (220-20)/(float)((s16)XY[1][0]-(s16)XY[2][0]);
			xoff_2 = (20*(s16)XY[1][0]-220*(s16)XY[2][0])/((s16)XY[1][0]-(s16)XY[2][0]);
			yfac_2 = (300-20)/(float)((s16)XY[2][1]-(s16)XY[1][1]);
			yoff_2 = (20*(s16)XY[2][1]-300*(s16)XY[1][1])/((s16)XY[2][1]-(s16)XY[1][1]);

			Pen_Point.xfac = (xfac_1 + xfac_2)/2;
			Pen_Point.yfac = (yfac_1 + yfac_2)/2;
			Pen_Point.xoff = (xoff_1 + xoff_2)/2;
			Pen_Point.yoff = (yoff_1 + yoff_2)/2;
			LCD_Clear(BLACK);//����
			break;											
		}	
	}

}

//���ô�����IO��
//PC0:TCLK SPIʱ��
//PC1:PEN  �����ж�
//PC2:TDIN SPI���� 
//PC3:DOUT SPI���
//PC13:TCS SPIƬѡ 
void Touch_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);//ʹ��IO��ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_13;//0 3 13������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;//1 2 ��������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		 

	//�����ⲿ�ж�ͨ�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�����ж�   
	NVIC_Init(&NVIC_InitStructure);//�����ж����ȼ�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);//ѡ�� GPIO�ܽ������ⲿ�ж���·  
	//�����ⲿ�ж�
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//�ⲿ��·EXIT1 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//���� EXTI��·Ϊ�ж����� 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//����������·�½���Ϊ�ж�����  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ���ⲿ�ж���״̬
	EXTI_Init(&EXTI_InitStructure);//�����ⲿ�ж�


////////////////////������У׼/////////////////////////
#ifdef ADJ_SAVE_ENABLE
	AT24C02_Init();
	if(Get_Adjdata()) return;//�Ѿ�У׼
	else
	{
		delay_ms(1000);
		LCD_Clear(BLACK);//����
		Touch_Adjust();  //��ĻУ׼	
		Save_Adjdata();
	}
	Get_Adjdata();
#else
	LCD_Clear(BLACK);//����
    Touch_Adjust();  //��ĻУ׼,���Զ�����
#endif
//////////////////////////////////////////////////
	 		
}

