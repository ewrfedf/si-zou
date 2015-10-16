#include "TFT.h"
#include "font.h"

u16  POINT_COLOR;
u16  BACK_COLOR;

/*
 * ��������LCD_WR_DATA
 * ����  ��TFT_LCDд�����ݡ�����
 * ����  ��data
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
__inline void LCD_WR_DATA(u16 data)
{
	LCD_RS_SET;
	LCD_CS_CLR;
	DATAOUT(data);
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
}

/*
 * ��������LCD_WR_REG
 * ����  ��TFT_LCDд���Ĵ���������
 * ����  ��data
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
__inline void LCD_WR_REG(u8 data)
{ 
	LCD_RS_CLR;//д��ַ  
 	LCD_CS_CLR; 
	DATAOUT(data); 
	LCD_WR_CLR; 
	LCD_WR_SET; 
 	LCD_CS_SET;   
}

/*
 * ��������LCD_WriteReg
 * ����  ��д�Ĵ���
 * ����  ��LCD_Reg�Ĵ�����ַ LCD_RegValueд��Ĵ�����ֵ
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*
 * ��������LCD_ReadReg
 * ����  �����Ĵ���
 * ����  ��LCD_Reg�Ĵ�����ַ
 * ���  ���Ĵ�����ֵ
 * ����  ��TFT.c�ڲ�����
 */
u16 LCD_ReadReg(u8 LCD_Reg)
{
	u16 t;
	LCD_WR_REG(LCD_Reg);//д��Ҫ��ȡ�ļĴ�����ֵ
	GPIOB->CRL=0X88888888; //PB0-7  ��������
	GPIOB->CRH=0X88888888; //PB8-15 ��������
	GPIOB->ODR=0XFFFF;    //ȫ�������	

	LCD_RS_SET;
	LCD_CS_CLR;
	//��ȡ����(���Ĵ���ʱ,������Ҫ��2��)
	LCD_RD_CLR;
	delay_us(5);//FOR 8989,��ʱ5us
	LCD_RD_SET;
	t=DATAIN;  
	LCD_CS_SET;

	GPIOB->CRL=0X33333333; //PB0-7  �������
	GPIOB->CRH=0X33333333; //PB8-15 �������
	GPIOB->ODR=0XFFFF;    //ȫ�������
	return t;
		
}

/*
 * ��������LCD_WriteRAM_Prepare
 * ����  ��׼����ʼдGRAM(��ַ�������Զ�+1)
 * ����  ����
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(R34);
}

/*
 * ��������LCD_WriteRAM
 * ����  ��LCDдGRAM
 * ����  ��RGB_Code
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
void LCD_WriteRAM(u16 RGB_Code)
{
	LCD_WR_DATA(RGB_Code);//дʮ��λGRAM	
}

/*
 * ��������LCD_BGR2RGB
 * ����  ��GBR ת RGB
 * ����  ��GBR��ʽ������
 * ���  ����
 * ����  ��TFT.c�ڲ�����
 */
u16 LCD_BGR2RGB(u16 c)
{
	u16 r,g,b,rgb;
	b = (c>>0) & 0x1F;
	g = (c>>5) & 0x3F;
	r = (c>>11) & 0x1F;
	rgb = (b<<11)+(g<<5)+(r<<0);
	return(rgb);		
}





/*
 * ��������LCD_Init
 * ����  ��LCD��ʼ��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;//����IO������ �ṹ��
	u16 DeviceCode;//ID��

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//ʹ��GPIOCʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//��������ʱ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//JTAG-DPʧ�� + SW-DPʹ��(PA13 PA14)

	//ѡ������6-10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO����ٶ�50Hz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);//����GPIOC 6-10����
	GPIO_SetBits(GPIOC,GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;//PB��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//����GPIOB ��������
	GPIO_SetBits(GPIOB,GPIO_Pin_All);
	
	
	
	delay_ms(50); // delay 50 ms
	LCD_WriteReg(0x00,0x0001);//������
	delay_ms(50); // delay 50 ms
	DeviceCode = LCD_ReadReg(0x00);//��ȡID��
	printf("LCD ID:%x\n",DeviceCode); //��ӡLCD ID
	
	/* ---ID 9325��������--- */
	LCD_WriteReg(0xE5,0x78F0); 
	LCD_WriteReg(0x01,0x0100); 
	LCD_WriteReg(0x02,0x0700); 
	LCD_WriteReg(0x03,0x1030); 
	LCD_WriteReg(0x04,0x0000); 
	LCD_WriteReg(0x08,0x0202);  
	LCD_WriteReg(0x09,0x0000);
	LCD_WriteReg(0x0A,0x0000); 
	LCD_WriteReg(0x0C,0x0000); 
	LCD_WriteReg(0x0D,0x0000);
	LCD_WriteReg(0x0F,0x0000);
	//power on sequence VGHVGL
	LCD_WriteReg(0x10,0x0000);   
	LCD_WriteReg(0x0011,0x0007);  
	LCD_WriteReg(0x0012,0x0000);  
	LCD_WriteReg(0x0013,0x0000); 
	LCD_WriteReg(0x0007,0x0000); 
	//vgh 
	LCD_WriteReg(0x0010,0x1690);   
	LCD_WriteReg(0x0011,0x0227);
	//delayms(100);
	//vregiout 
	LCD_WriteReg(0x0012,0x009D); //0x001b
	//delayms(100); 
	//vom amplitude
	LCD_WriteReg(0x0013,0x1900);
	//delayms(100); 
	//vom H
	LCD_WriteReg(0x0029,0x0025); 
	LCD_WriteReg(0x002B,0x000D); 
	//gamma
	LCD_WriteReg(0x0030,0x0007);
	LCD_WriteReg(0x0031,0x0303);
	LCD_WriteReg(0x0032,0x0003);// 0006
	LCD_WriteReg(0x0035,0x0206);
	LCD_WriteReg(0x0036,0x0008);
	LCD_WriteReg(0x0037,0x0406); 
	LCD_WriteReg(0x0038,0x0304);//0200
	LCD_WriteReg(0x0039,0x0007); 
	LCD_WriteReg(0x003C,0x0602);// 0504
	LCD_WriteReg(0x003D,0x0008); 
	//ram
	LCD_WriteReg(0x0050,0x0000); 
	LCD_WriteReg(0x0051,0x00EF);
	LCD_WriteReg(0x0052,0x0000); 
	LCD_WriteReg(0x0053,0x013F);  
	LCD_WriteReg(0x0060,0xA700); 
	LCD_WriteReg(0x0061,0x0001); 
	LCD_WriteReg(0x006A,0x0000); 
	//
	LCD_WriteReg(0x0080,0x0000); 
	LCD_WriteReg(0x0081,0x0000); 
	LCD_WriteReg(0x0082,0x0000); 
	LCD_WriteReg(0x0083,0x0000); 
	LCD_WriteReg(0x0084,0x0000); 
	LCD_WriteReg(0x0085,0x0000); 
	//
	LCD_WriteReg(0x0090,0x0010); 
	LCD_WriteReg(0x0092,0x0600); 
		
	LCD_WriteReg(0x0007,0x0133);
	LCD_WriteReg(0x00,0x0022);//
	/* ----------------------- */
		
	POINT_COLOR = WHITE;
	BACK_COLOR = BLACK;	 
	LCD_Clear(BACK_COLOR);
	LCD_LED(1);//��������
}

/*
 * ��������LCD_DisplayOn
 * ����  ��LCD������ʾ
 * ����  ����
 * ���  ����
 * ����  ����/�ⲿ����
 */
void LCD_DisplayOn(void)
{
	LCD_WriteReg(R7, 0x0173); //26��ɫ��ʾ����	
}

/*
 * ��������LCD_DisplayOff
 * ����  ��LCD�ر���ʾ
 * ����  ����
 * ���  ����
 * ����  ����/�ⲿ����
 */
void LCD_DisplayOff(void)
{
	LCD_WriteReg(R7, 0x0);//�ر���ʾ	
} 

/*
 * ��������LCD_Clear
 * ����  ��LCD��������
 * ����  ����
 * ���  ��Color Ҫ���������ɫ
 * ����  ����/�ⲿ����
 */
void LCD_Clear(u16 Color)
{
	u32 index=0;      
	LCD_SetCursor(0x00,0x0000); //���ù��λ�� 
	LCD_WriteRAM_Prepare(); //��ʼд��GRAM	 	  
	for(index=76800;index>0;index--)
	{
		LCD_WR_DATA(Color);   
	}	
}

/*
 * ��������LCD_SetCursor
 * ����  �����ù��λ��(__inline��궨�������ͬ,�ӿ������ٶ�)
 * ����  ��Xpos X�� Ypos Y��
 * ���  ����
 * ����  ���ڲ�����
 */
__inline void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);		
}

/*
 * ��������LCD_DrawPoint
 * ����  ����1��
 * ����  ��x X������ y Y������
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//���ù��λ�� 
	LCD_WR_REG(R34);//��ʼд��GRAM
	LCD_WR_DATA(POINT_COLOR); 	
}

/*
 * ��������LCD_ReadPoint
 * ����  ����1��
 * ����  ��x X������ y Y������
 * ���  ���˵����ɫRGB
 * ����  ���ⲿ����
 */
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 t;	
	if(x>=LCD_W||y>=LCD_H)return 0;//�����˷�Χ,ֱ�ӷ���	
	LCD_SetCursor(x,y);
	LCD_WR_REG(R34);//ѡ��GRAM��ַ

	GPIOB->CRL=0X88888888; //PB0-7  ��������
	GPIOB->CRH=0X88888888; //PB8-15 ��������
	GPIOB->ODR=0XFFFF;     //ȫ�������

	LCD_RS_SET;
	LCD_CS_CLR;
	//��ȡ����(��GRAMʱ,��Ҫ��2��)
	LCD_RD_CLR;					   
	LCD_RD_SET;
	delay_us(2);//FOR 9320,��ʱ2us
	//dummy READ
	LCD_RD_CLR;					   
	delay_us(2);//FOR 8989,��ʱ2us					   
	LCD_RD_SET;
	t=DATAIN;  
	LCD_CS_SET;

	GPIOB->CRL=0X33333333; //PB0-7  �������
	GPIOB->CRH=0X33333333; //PB8-15 �������
	GPIOB->ODR=0XFFFF;    //ȫ�������
	return t;
	//return LCD_BGR2RGB(t);		
}

/*
 * ��������LCD_Fill
 * ����  ����ָ�����������ָ����ɫ 
 *         �����С:(xend-xsta)*(yend-ysta)
 * ����  ��xsta	ysta xend yend color
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{
	u16 i;
	u32 point_num;

	//����д����������
	LCD_WriteReg(R80,xsta);
	LCD_WriteReg(R81,xend);
	LCD_WriteReg(R82,ysta);
	LCD_WriteReg(R83,yend);
	point_num = (xend-xsta+1)*(yend-ysta+1);
	
	LCD_SetCursor(xsta,ysta); //���ù��λ�� 
	LCD_WriteRAM_Prepare(); //��ʼд��GRAM
	for(i=point_num;i>0;i--)
	{
		LCD_WR_DATA(color);	
	}
	LCD_WriteReg(R80,0);
	LCD_WriteReg(R81,240);
	LCD_WriteReg(R82,0);
	LCD_WriteReg(R83,320);	
}


/*
 * ��������LCD_ShowChar
 * ����  ����ָ��λ����ʾһ���ַ�
 *		   x,y:���������
 *		   num:Ҫ��ʾ���ַ� 
 *         size:�����С 12/16
 *		   mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
 * ����  ��x y num size mode
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{
#define MAX_CHAR_POSX 231
#define MAX_CHAR_POSY 303

	u8 temp;//��ȡ�ַ���
    u8 i,j;//ѭ������
	if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;//�ж��Ƿ����	
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	
	if(mode == 0)//�ǵ���ģʽ
	{
		//����д����������
		LCD_WriteReg(R80,x);
		LCD_WriteReg(R82,y);
		if(size == 16)
		{
			LCD_WriteReg(R81,x+8-1);
			LCD_WriteReg(R83,y+16-1);
		}
		else if(size == 12)
		{
			LCD_WriteReg(R81,x+6-1);
			LCD_WriteReg(R83,y+12-1);
		}
		LCD_SetCursor(x,y);//���ù��λ�� 
		LCD_WriteRAM_Prepare();//��ʼд��GRAM

		for(i=0;i<size;i++)
		{
			if(size == 16) temp=asc2_1608[num][i];//����1608����
			else if (size == 12) temp=asc2_1206[num][i];//����1206���� 
			for(j=0;j<size/2;j++)
			{
				if(temp&(0x01<<j)) LCD_WR_DATA(POINT_COLOR);
				else LCD_WR_DATA(BACK_COLOR);		
			}
		}
		//��������
		LCD_WriteReg(R80,0);
		LCD_WriteReg(R81,240);
		LCD_WriteReg(R82,0);
		LCD_WriteReg(R83,320);
	 }
	 else//����ģʽ
	 {
	 	for(i=0;i<size;i++)
		{
			if(size == 16) temp=asc2_1608[num][i];//����1608����
			else if (size == 12) temp=asc2_1206[num][i];//����1206���� 
			for(j=0;j<size/2;j++)
			{
				if(temp&(0x01<<j)) LCD_DrawPoint(x+j,y+i);//��һ����		
			}
		}	
	 }					
}

/*
 * ��������LCD_ShowNum
 * ����  ����ʾ����
 *		   x,y:���������
 *		   num:Ҫ��ʾ������ 
 *		   len:��ʾ���ֵĳ���
 *         size:�����С 12/16
 *		   mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
 * ����  ��x y num len size mode
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{
	u8 i;
	u32 temp = 1;
	u32 n;
	temp = pow(10,len-1);//��ʾ��λ��������	
	n = num%(temp*10);//ȥ����������λ

	for(i=0;i<len;i++)
	{
		LCD_ShowChar(x+8*i,y,(n/temp)+'0',size,mode);
		n = n%temp;
		temp = temp/10;			
	}		
}

/*
 * ��������LCD_ShowString
 * ����  ����ʾһ���ַ���
 *		   x,y:���������
 *		   *p:�ַ����׵�ַ
 *		   mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
 * ����  ��x y *p mode
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_ShowString(u16 x,u16 y,const u8 *p,u8 mode)
{
	while(*p != '\0')
	{
		if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(BLACK);}
		LCD_ShowChar(x,y,*p,16,mode);
		x += 8; 
		p++;	
	}	
}


/*
 * ��������LCD_ShowFloatNum
 * ����  ����ʾ��С��
 *		   x,y:���������
 *		   num:С����ֵ
 *		   fl_n:С��λ��
 *         nu_n:����λ��
 *		   mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
 * ����  ��x y *p mode
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_ShowFloatNum(u16 x,u16 y,float num,u8 nu_n,u8 fl_n,u8 size,u8 mode)
{
	u16 i;
	u16 temp1 = (u16)num;//��������
	float temp2;//С������
	
	i = pow(10,fl_n);
	temp2 = (num-(u16)num)*i;
	
	LCD_ShowNum(x,y,temp1,nu_n,size,mode);//��ʾ��������
	LCD_ShowChar(x+nu_n*8,y,'.',size,mode);//��ʾһ���ַ�
	LCD_ShowNum(x+nu_n*8+8,y,temp2,fl_n,size,mode);//��ʾ��������
}



/*
 * ��������Draw_Circle
 * ����  ����Բ(��ͨ�㷨-�ⷽ��)
 * ����  ��x0 X������ y0 Y������ rԲ�İ뾶
 * ���  ����
 * ����  ���ⲿ����
 */
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	u16 i;
	u16 temp1,temp2;
	
	for(i=x0-r;i<=x0;i++)
	{
		temp1 = y0+sqrtf(r*r-((s16)i-x0)*((s16)i-x0));
		temp2 = y0-sqrtf(r*r-((s16)i-x0)*((s16)i-x0));
		LCD_DrawPoint(i,temp1);
		LCD_DrawPoint(i,temp2);
		LCD_DrawPoint(2*x0-i,temp1);
		LCD_DrawPoint(2*x0-i,temp2);
	}
 	
	for(i=y0-r;i<=y0;i++)
	{
		temp1 = x0+sqrtf(r*r-((s16)i-y0)*((s16)i-y0));
		temp2 = x0-sqrtf(r*r-((s16)i-y0)*((s16)i-y0));
		LCD_DrawPoint(temp1,i);
		LCD_DrawPoint(temp2,i);
		LCD_DrawPoint(temp1,2*y0-i);
		LCD_DrawPoint(temp2,2*y0-i);	
	}			
}


/*
 * ��������LCD_DrawLine
 * ����  ������(��ͨ�㷨-�ⷽ��)
 * ����  ��x1 y1 ��ʼ���� x2 y2��������
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 i;
	float k_x,b_x;
	float k_y,b_y;

	if(x1 == x2) //б�������
	{
		if(y1>y2)
		{
			for(i=y2;i<=y1;i++) LCD_DrawPoint(x1,i); 
		}
		else
		{
			for(i=y1;i<=y2;i++) LCD_DrawPoint(x1,i);	
		}	
	}
	else if(y1 == y2)//б��Ϊ0
	{
		if(x1>x2)
		{
			for(i=x2;i<=x1;i++) LCD_DrawPoint(i,y1);  
		}
		else
		{
			for(i=x1;i<=x2;i++) LCD_DrawPoint(i,y1);  	
		}
	}
	else
	{
		k_x = ((float)y2-y1)/((float)x2-x1);//xΪ�Ա���ʱ��б��
		b_x = (float)((y1*x2)-(float)(y2*x1))/(float)(x2-x1);//xΪ�Ա���ʱ�ĳ���
		
		k_y = 1/k_x;//yΪ�Ա���ʱ��б��
		b_x = -b_x/k_x;//yΪ�Ա���ʱ�ĳ���

		if(fabsf(x2-x1)>fabsf(y2-y1))
		{
			if(x1>x2)
			{
				for(i=x2;i<=x1;i++) LCD_DrawPoint(i,(i*k_x+b_x));  
			}
			else
			{
				for(i=x1;i<=x2;i++) LCD_DrawPoint(i,(i*k_x+b_x)); 
			}	
		}
		else
		{
			if(y1>y2)
			{
				for(i=y2;i<=y1;i++) LCD_DrawPoint((i*k_y+b_y),i); 
			}
			else
			{
				for(i=y1;i<=y2;i++) LCD_DrawPoint((i*k_y+b_y),i);	
			}	
		}		
	}
}

/*
 * ��������LCD_DrawRectangle
 * ����  ����������
 * ����  ��x1 y1 ��ʼ���� x2 y2��������
 * ���  ����
 * ����  ���ⲿ����
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x1,y2);//��2��ֱ��
	LCD_DrawLine(x2,y1,x2,y2);
	LCD_DrawLine(x1,y1,x2,y1);//��2����
	LCD_DrawLine(x1,y2,x2,y2);			
}



