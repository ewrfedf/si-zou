#ifndef __W25X16_H__
#define __W25X16_H__

#include "stm32f10x.h"
#include "spi.h"
#include "SysTick.h"

//CSѡ��FLASH
/*
#define	SPI_FLASH_CS(a) if(a)\
					    GPIO_SetBits(GPIOA,GPIO_Pin_2);\
				        else\
					    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
*/
#define SPI_FLASH_CS_H 	GPIOA->BSRR = GPIO_Pin_2
#define SPI_FLASH_CS_L	GPIOA->BRR = GPIO_Pin_2


#define W25Q16 	0XEF14					 
#define SPI_FLASH_TYPE	W25Q16
//W25X16��д
#define FLASH_ID 0XEF14

//ָ���
#define W25X_WriteEnable		0x06//дʹ��
#define W25X_WriteDisable		0x04//д���� 
#define W25X_ReadStatusReg		0x05//��״̬�Ĵ���
#define W25X_WriteStatusReg		0x01//д״̬�Ĵ��� 
#define W25X_ReadData			0x03//������
#define W25X_FastReadData		0x0B//��� 
#define W25X_FastReadDual		0x3B//���˫��� 
#define W25X_PageProgram		0x02//ҳ��� 
#define W25X_BlockErase			0xD8//�����(64K) 
#define W25X_SectorErase		0x20//��������(4K) 
#define W25X_ChipErase			0xC7//оƬ����
#define W25X_PowerDown			0xB9//����
#define W25X_ReleasePowerDown	0xAB//�ͷŵ��� 
#define W25X_DeviceID			0xAB// ����ID
#define W25X_ManufactDeviceID	0x90//����/����ID 
#define W25X_JedecDeviceID		0x9F// JEDEC ID

void SPI_Flash_Init(void);
u16  SPI_Flash_ReadID(void);  	    //��ȡFLASH ID
u8	 SPI_Flash_ReadSR(void);        //��ȡ״̬�Ĵ���
void SPI_Flash_Wait_Busy(void);     //�ȴ����� 
void SPI_FLASH_Write_SR(u8 sr);  	//д״̬�Ĵ���
void SPI_FLASH_Write_Enable(void);  //дʹ�� 
void SPI_FLASH_Write_Disable(void);	//д����
void SPI_Flash_Erase_Chip(void);    	  //��Ƭ����
void SPI_Flash_Erase_Sector(u32 Dst_Addr);//��������
void SPI_Flash_PowerDown(void);           //�������ģʽ
void SPI_Flash_WAKEUP(void);			  //����

void SPI_Flash_Init(void);//��ʼ��Flash
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
//�޼���дSPI FLASH
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);

void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //��ȡflash
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//д��flash










#endif

