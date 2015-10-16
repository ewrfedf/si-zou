#ifndef __24C02_H__
#define __24C02_H__

#include "stm32f10x.h"
#include "myiic.h"


void AT24C02_Init(void);//��ʼ��IIC�ӿ�
					  
void AT24C02_WriteOneByte(u8 WriteAddr,u8 DataToWrite);		//ָ����ַд��һ���ֽ�
u8 AT24C02_ReadOneByte(u8 ReadAddr);							//ָ����ַ��ȡһ���ֽ�

void AT24C02_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 AT24C02_ReadLenByte(u8 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������

void AT24C02_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void AT24C02_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����

u8 AT24C02_Check(void);  //�������
void AT24C02_Init(void); //��ʼ��IIC








#endif


