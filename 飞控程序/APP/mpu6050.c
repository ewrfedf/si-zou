#include "mpu6050.h"
#include "myiic.h"
#include "led.h"
#include "SysTick.h"
#include  <math.h> 

u8 	mpu6050_buffer[14];	
u8 	BUF[14];	

S_INT16_XYZ 	GYRO_OFFSET,ACC_OFFSET;//陀螺仪和加速度计偏移		
S_INT16_XYZ 	MPU6050_ACC_LAST,MPU6050_GYRO_LAST;//角速度和加速度	
	

/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
*******************************************************************************/
void MPU6050_READ(void)
{

	 BUF[0]=Single_Read(devAddr,MPU6050_RA_GYRO_XOUT_L); 
   BUF[1]=Single_Read(devAddr,MPU6050_RA_GYRO_XOUT_H);
   MPU6050_GYRO_LAST.X=	(BUF[1]<<8)|BUF[0];
   MPU6050_GYRO_LAST.X/=16.4; 						   //读取计算X轴数据

   BUF[2]=Single_Read(devAddr,MPU6050_RA_GYRO_YOUT_L);
   BUF[3]=Single_Read(devAddr,MPU6050_RA_GYRO_YOUT_H);
   MPU6050_GYRO_LAST.Y=	(BUF[3]<<8)|BUF[2];
   MPU6050_GYRO_LAST.Y/=16.4; 						   //读取计算Y轴数据
   BUF[4]=Single_Read(devAddr,MPU6050_RA_GYRO_ZOUT_L);
   BUF[5]=Single_Read(devAddr,MPU6050_RA_GYRO_ZOUT_H);
   MPU6050_GYRO_LAST.Z=	(BUF[5]<<8)|BUF[4];
   MPU6050_GYRO_LAST.Z/=16.4; 					       //读取计算Z轴数据
   
	 BUF[6]=Single_Read(devAddr,MPU6050_RA_ACCEL_XOUT_L); 
   BUF[7]=Single_Read(devAddr,MPU6050_RA_ACCEL_XOUT_H);
   MPU6050_ACC_LAST.X=	(BUF[7]<<8)|BUF[6];
   MPU6050_ACC_LAST.X/=163.84; 						   //读取计算X轴数据

   BUF[8]=Single_Read(devAddr,MPU6050_RA_ACCEL_YOUT_L);
   BUF[9]=Single_Read(devAddr,MPU6050_RA_ACCEL_YOUT_H);
   MPU6050_ACC_LAST.Y=	(BUF[9]<<8)|BUF[8];
   MPU6050_ACC_LAST.Y/=163.84; 						   //读取计算Y轴数据
   BUF[10]=Single_Read(devAddr,MPU6050_RA_ACCEL_ZOUT_L);
   BUF[11]=Single_Read(devAddr,MPU6050_RA_ACCEL_ZOUT_H);
   MPU6050_ACC_LAST.Z=	(BUF[11]<<8)|BUF[10];
   MPU6050_ACC_LAST.Z/=163.84; 					       //读取计算Z轴数据
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
u8 MPU6050_INIT(void)
{
	u8 ack;
	
	ack = Single_Read(devAddr, 0x75);
	if (!ack)
     return FALSE;
	//LED0(0);
	Single_Write(devAddr, MPU6050_RA_PWR_MGMT_1, 0x00);  	//解除休眠状态
	Single_Write(devAddr, MPU6050_RA_SMPLRT_DIV, 0x07);     
	Single_Write(devAddr, MPU6050_RA_CONFIG, 0x03);              //低通滤波
	Single_Write(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);  //陀螺仪量程 +-1000
	Single_Write(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2);   //加速度量程 +-4G
	return TRUE;	
}


//加速度计水平校准
void ACC_SET_OFFSET(void)
{
	
}

//陀螺仪零偏校准
void GYRO_SET_OFFSET(void)
{
	u8 times=0;
	s32 gyro_x=0,gyro_y=0,gyro_z=0;
	
	
	//陀螺仪零偏校准  --  START  --
	for(times=0;times<100;times++)
	{
		MPU6050_READ();//读取
		
		gyro_x += MPU6050_GYRO_LAST.X;
		gyro_y += MPU6050_GYRO_LAST.Y;
		gyro_z += MPU6050_GYRO_LAST.Z;
		delay_ms(5);
	}
	
	GYRO_OFFSET.X = gyro_x/100;
	GYRO_OFFSET.Y = gyro_y/100;
	GYRO_OFFSET.Z = gyro_z/100;
	//陀螺仪零偏校准  --  END  --
	
}


