#include "W25X16.h"

//W25X16
//256���ֽ�Ϊһҳ
//16ҳΪһ������Sectors(4K)
//16������Ϊ1������Block(64K)
//����Ϊ2M�ֽ�,����32������,512������,8192��ҳ


/*
 * ��������SPI_Flash_Init
 * ����  ����ʼ��SPI FLASH��IO��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//GPIOA ʱ��ʹ�� 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  //SPI CS
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4);
	
	SPIx_SetSpeed(SPI_BaudRatePrescaler_4);//4��Ƶ
	SPIx_Init();	
}


/*
 * ��������SPI_Flash_ReadID
 * ����  ����ȡоƬID W25X16��ID:0xEF14
 * ����  ����
 * ���  ��оƬ��ID��
 * ����  ���ⲿ����
 */
u16  SPI_Flash_ReadID(void)
{
	u16 temp = 0;
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ManufactDeviceID);
	SPIx_ReadWriteByte(0x00);
	SPIx_ReadWriteByte(0x00);
	SPIx_ReadWriteByte(0x00);
	temp |= (SPIx_ReadWriteByte(0xFF)<<8);
	temp |= SPIx_ReadWriteByte(0xFF);	
	SPI_FLASH_CS_H;
	return temp;	
}




/*
 * ��������SPI_Flash_ReadSR
 * ����  ����ȡSPI_FLASH��״̬�Ĵ���
		   BIT7  6   5   4   3   2   1   0
		   SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 * ����  ����
 * ���  ��״̬�Ĵ�����ֵ
 		   SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
		   TB,BP2,BP1,BP0:FLASH����д��������
		   WEL:дʹ������
		   BUSY:æ���λ(1,æ;0,����)
		   Ĭ��:0x00
 * ����  ���ⲿ����
 */
u8 SPI_Flash_ReadSR(void)
{
	u8 byte;
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ReadStatusReg);
	byte = SPIx_ReadWriteByte(0xFF);
	SPI_FLASH_CS_H;
	return byte;		
}


/*
 * ��������SPI_Flash_Wait_Busy
 * ����  ���ȴ�����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Wait_Busy(void)
{ 
	while(SPI_Flash_ReadSR()&0x01);
}


/*
 * ��������SPI_FLASH_Write_SR
 * ����  ��SPI_FLASH״̬�Ĵ���(ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!)
 * ����  ��д����ֵ
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_FLASH_Write_SR(u8 sr)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteStatusReg);
	SPIx_ReadWriteByte(sr);	
	SPI_FLASH_CS_H;	
}

/*
 * ��������SPI_FLASH_Write_Enable
 * ����  ��SPI_FLASHдʹ��(��WEL��λ)
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_FLASH_Write_Enable(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteEnable);
	SPI_FLASH_CS_H;	
}


/*
 * ��������SPI_FLASH_Write_Disable
 * ����  ��SPI_FLASHд��ֹ(��WEL����)
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_FLASH_Write_Disable(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteDisable);
	SPI_FLASH_CS_H;		
}

/*
 * ��������SPI_Flash_Erase_Chip
 * ����  ����������оƬ 25s 
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Erase_Chip(void)
{
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ChipErase);
	SPI_FLASH_CS_H;
	SPI_Flash_Wait_Busy();
}


/*
 * ��������SPI_Flash_Erase_Sector
 * ����  ������һ������(����һ��ɽ��������ʱ��:150ms)
 * ����  ��Dst_Addr:������ַ 0~511 for w25x16
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Erase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 4096;
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_SectorErase);
	SPIx_ReadWriteByte((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    SPIx_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPIx_ReadWriteByte((u8)Dst_Addr);
	SPI_FLASH_CS_H;	
	SPI_Flash_Wait_Busy();
}


/*
 * ��������SPI_Flash_PowerDown
 * ����  ���������ģʽ
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_PowerDown(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_PowerDown);
	SPI_FLASH_CS_H;
	delay_us(3);//�ȴ�TPD 		
}


/*
 * ��������SPI_Flash_WAKEUP
 * ����  ������
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_WAKEUP(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ReleasePowerDown);
	SPI_FLASH_CS_H;
	delay_us(3);//�ȴ�TRES1	
}



/*
 * ��������SPI_Flash_Write_Page
 * ����  ��SPI��һҳ(0~65535)��д������256���ֽڵ�����
 * ����  ��pBuffer:���ݴ洢��
		   WriteAddr:��ʼд��ĵ�ַ 0~1677216(24bit)
           NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u16 i;
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_PageProgram);
	SPIx_ReadWriteByte((u8)((WriteAddr)>>16));  //����24bit��ַ    
    SPIx_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPIx_ReadWriteByte((u8)WriteAddr);

	for(i=0;i<NumByteToWrite;i++)
	{
		SPIx_ReadWriteByte(pBuffer[i]);
	}
	SPI_FLASH_CS_H;
	SPI_Flash_Wait_Busy();	
}

/*
 * ��������SPI_Flash_Write_NoCheck
 * ����  ���޼���дSPI FLASH �����Զ���ҳ����(����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!)
 * ����  ��pBuffer:���ݴ洢��
		   WriteAddr:��ʼд��ĵ�ַ 0~1677216(24bit)
           NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
 * ���  ��pBuffer:���ݴ洢��
	       WriteAddr:��ʼд��ĵ�ַ(24bit)
		   NumByteToWrite:Ҫд����ֽ���(���65535)
 * ����  ���ⲿ����
 */
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	/*
	u16 pagermain;
	pagermain = 256 - (WriteAddr%256);//��ҳʣ����ֽ���
	if(NumByteToWrite <= pagermain) pagermain = NumByteToWrite;//������256���ֽ�
	while(1)
	{
		SPI_Flash_Write_Page(pBuffer,WriteAddr,pagermain);
		if(pagermain == NumByteToWrite) break;//д�������
		else
		{
			pBuffer = pBuffer + pagermain;
			WriteAddr = WriteAddr + pagermain;
			
			NumByteToWrite = NumByteToWrite - pagermain;//��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256) pagermain = 256;//һ�ο���д��256���ֽ�
			else pagermain = NumByteToWrite;//����256���ֽ��� 			
		}		
	}
	*/
	u16 x;
	x = 256 - (WriteAddr%256);//��һ��д����������
	while(1)
	{
		if(NumByteToWrite > x)
		{
			
				SPI_Flash_Write_Page(pBuffer,WriteAddr,x);
				NumByteToWrite = NumByteToWrite - x;
				pBuffer = pBuffer + x;
				WriteAddr = WriteAddr + x;
				
				x = 256;//�Ժ�ÿ��д����������λ256					
				
		}
		else
		{
			SPI_Flash_Write_Page(pBuffer,WriteAddr,NumByteToWrite);
			break;	
		}
	}
				
}



/*
 * ��������SPI_Flash_Read
 * ����  ����ָ����ַ��ʼ��ȡָ�����ȵ�����
 * ����  ��pBuffer:���ݴ洢��
		   ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
 		   NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)
{
	u16 i;
	SPI_FLASH_CS_L;                            //ʹ������   
    SPIx_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
    SPIx_ReadWriteByte((u8)((ReadAddr)>>16));  //����24bit��ַ    
    SPIx_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPIx_ReadWriteByte((u8)ReadAddr);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i] = SPIx_ReadWriteByte(0xFF);//ѭ������		
	}
	SPI_FLASH_CS_H; 	
}



/*
 * ��������SPI_Flash_Write
 * ����  ��дSPI FLASH ��ָ����ַ��ʼд��ָ�����ȵ�����(д��֮ǰ���Ȳ�����Ҫд�������!!!!) 
 * ����  ��pBuffer:���ݴ洢��
		   WriteAddr:��ʼд��ĵ�ַ(24bit)
           NumByteToWrite:Ҫд����ֽ���(���65535)
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u16 startSector,endSector;
	u16 i;
	startSector = (WriteAddr/4096);  //��ʼ��������
	endSector = ((WriteAddr+NumByteToWrite)/4096)+1; //������������
	for(i=startSector;i<=endSector;i++) //����Ҫд�������
	{
		SPI_Flash_Erase_Sector(i);	
	}
	
	SPI_Flash_Write_NoCheck(pBuffer,WriteAddr,NumByteToWrite);//д������			
}






