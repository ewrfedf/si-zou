#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "stm32f10x.h"
#include "SysTick.h"
#include "math.h"
#include "24c02.h"
#include "TFT.h"


//����״̬	 
#define Key_Down 0x01
#define Key_Up   0x00 

//�ʸ˽ṹ��
typedef struct 
{
	u16 X0;//ԭʼ����
	u16 Y0;
	u16 X; //����/�ݴ�����
	u16 Y;						   	    
	u8  Key_Sta;//�ʵ�״̬			  
//������У׼����
	float xfac;
	float yfac;
	s16 xoff;
	s16 yoff;
//�����Ĳ���,��������������������ȫ�ߵ�ʱ��Ҫ�õ�.
//touchtype=0��ʱ��,�ʺ�����ΪX����,����ΪY�����TP.
//touchtype=1��ʱ��,�ʺ�����ΪY����,����ΪX�����TP.
	u8 touchtype;
}Pen_Holder;	   
extern Pen_Holder Pen_Point;

//�봥����оƬ��������


#define PEN_read   GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)
#define DOUT_read  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)
/*	   
#define PEN_read  GPIOC->IDR  & GPIO_Pin_1   //PC1  INT
#define DOUT_read GPIOC->IDR  & GPIO_Pin_2   //PC2  MISO
*/
#define TDIN_H  GPIOC->BSRR = GPIO_Pin_3  //PC3  MOSI_H
#define TDIN_L  GPIOC->BRR = GPIO_Pin_3  //PC3  MOSI_L

#define TCLK_H GPIOC->BSRR = GPIO_Pin_0  //PC0  SCLK_H
#define TCLK_L GPIOC->BRR = GPIO_Pin_0  //PC0  SCLK_L

#define TCS_H  GPIOC->BSRR = GPIO_Pin_13 //PC13 TCS_H
#define TCS_L  GPIOC->BRR = GPIO_Pin_13 //PC13 TCS_H

    
//ADS7843/7846/UH7843/7846/XPT2046/TSC2046 ָ�
//#define CMD_RDX   0X90  //0B10010000���ò�ַ�ʽ��X����
//#define CMD_RDY	0XD0  //0B11010000���ò�ַ�ʽ��Y����
extern u8 CMD_RDX;
extern u8 CMD_RDY;
   											 
//#define TEMP_RD	0XF0  //0B11110000���ò�ַ�ʽ��Y����  

#define ADJ_SAVE_ENABLE //ʹ�ñ���	    
			  
void Touch_Init(void);		 //��ʼ��
u8 Read_ADS(u16 *x,u16 *y,u8 delay);	 //��������˫�����ȡ
u8 Read_ADS2(u16 *x,u16 *y,u8 delay); //����ǿ�˲���˫���������ȡ
u16 ADS_Read_XY(u8 xy,u8 delay);		 //���˲��������ȡ(������)
u16 ADS_Read_AD(u8 CMD);	 //��ȡADת��ֵ
void ADS_Write_Byte(u8 num); //�����оƬд��һ������
void Drow_Touch_Point(u8 x,u16 y);//��һ�������׼��
void Draw_Big_Point(u8 x,u16 y);  //��һ�����
void Touch_Adjust(void);          //������У׼
void Save_Adjdata(void);		  //����У׼����
u8 Get_Adjdata(void); 			  //��ȡУ׼����
void Pen_Int_Set(u8 en); 		  //PEN�ж�ʹ��/�ر�
void Convert_Pos(void);           //���ת������


#endif


