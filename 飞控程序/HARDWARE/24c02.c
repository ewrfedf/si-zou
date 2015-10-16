#include "24c02.h"


/*
 * ��������AT24C02_Init
 * ����  ��AT24C02��ʼ��(IO�ڳ�ʼ��)
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void AT24C02_Init(void)
{
	I2C_Init_IO();
}

/*
 * ��������AT24C02_WriteOneByte
 * ����  ����AT24C02ָ����ַд��һ������
 * ����  ��WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
 		   DataToWrite:Ҫд�������
 * ���  ����
 * ����  ���ⲿ����
 */
void AT24C02_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{
	Single_Write(WriteAddr,DataToWrite,0xA0);
}

/*
 * ��������AT24C02_ReadOneByte
 * ����  ����AT24C02ָ����ַ����һ������
 * ����  ��ReadAddr:��ʼ�����ĵ�ַ
 * ���  ������������
 * ����  ���ⲿ����
 */
u8 AT24C02_ReadOneByte(u8 ReadAddr)
{
	u8 databyte;
	databyte = Single_Read(ReadAddr,0xA0);
	return databyte;
		
}

//��AT24C02�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit�����ݣ�����
//�ȵ��ֽ� �ٸ��ֽڣ�����
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24C02_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{
	u8 i;
	for(i=0;i<Len;i++)
	{
		AT24C02_WriteOneByte(WriteAddr+i,(DataToWrite>>(8*i))&0xFF);	
	} 	
}


/*
 * ��������AT24C02_ReadLenByte
 * ����  ����AT24C02�����ָ����ַ��ʼ��������ΪLen������,�ú������ڶ���16bit����32bit������.
 		   ��ŷ�ʽΪ���ȵ��ֽ� �ٸ��ֽڣ�����
 * ����  ��ReadAddr   :��ʼ�����ĵ�ַ
 		   Len        :Ҫ�������ݵĳ���2,4
 * ���  ������������
 * ����  ���ⲿ����
 */
u32 AT24C02_ReadLenByte(u8 ReadAddr,u8 Len)
{
	u8 i;
	u32 databyte=0;
	for(i=0;i<Len;i++)
	{
		databyte <<= 8;
		databyte += AT24C02_ReadOneByte(ReadAddr+Len-i-1);	
	}
	return databyte;	
}


/*
 * ��������AT24C02_Write
 * ����  ����AT24C02����д������
 * ����  ��WriteAddr :��ʼд����׵�ַ
		   pBuffer   :���������׵�ַ
		   NumToWrite:Ҫд�����ݵĸ���
 * ���  ����
 * ����  ���ⲿ����
 */
void AT24C02_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite)
{
	u8 i;
	for(i=0;i<NumToWrite;i++)
	{
		AT24C02_WriteOneByte(WriteAddr+i,pBuffer[i]);	
	}	
}

//����д��AT24C02�������


/*
 * ��������AT24C02_Read
 * ����  ������д��AT24C02�������
 * ����  ��ReadAddr :��ʼ�������׵�ַ
		   pBuffer  :���������׵�ַ
           NumToRead:Ҫ�������ݵĸ���
 * ���  ����pBufferΪָ���һλ����
 * ����  ���ⲿ����
 */
void AT24C02_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead)
{
	u8 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i] = AT24C02_ReadOneByte(ReadAddr+i);	
	}
}












