#include "stm32f10x.h"
#include "SysTick.h"
#include "led.h"
#include "dma.h"
#include "myiic.h"
#include "stmflash.h"

#include "systime.h"
#include "mpu6050.h"
#include "imu.h"
#include "control.h"
#include "rc.h"
#include "motor.h"
#include "maincom.h"
#include "refreshled.h"




void FlyControlAdjust(void)
{
	u8 x_mode;
	Data_Send_PID(PID_ROL.P,PID_ROL.I,PID_ROL.D,PID_PIT.P,PID_PIT.I,PID_PIT.D,PID_YAW.P,PID_YAW.I,PID_YAW.D);//����PID����
	while(1)
	{
		//�ж��Ƿ�Ҫ����(ң��������)
		if(Is_Armed(RC_CH[3-1],RC_CH[4-1]) == 1) break;
	
		//(��λ������)
		x_mode = Recv_Command();
		if(x_mode == 1)//ң����ͨ��У׼
		{
			Channel_Adjust();
		}
		else if(x_mode == 2)//����λ������PID����
		{
			PID_ReadFlash();
			Data_Send_PID(PID_ROL.P,PID_ROL.I,PID_ROL.D,PID_PIT.P,PID_PIT.I,PID_PIT.D,PID_YAW.P,PID_YAW.I,PID_YAW.D);//����PID����
		}
		else if(x_mode == 3)//��λ�����ɿط���PID����
		{
			Get_Recv_PID();//����PIDֵ��RAM
			PID_WriteFlash();//����PIDֵ��Flash
		}
		
		LED0(ON);
		delay_ms(10);
		LED0(OFF);
		delay_ms(100);
	
	}
	
}

void FlyControlInit(void)
{
	//RCͨ����ʼ��
	if(Channel_Config() == 0)
	{
		while(1)
		{
			LED0(ON);
			delay_ms(500);
			LED0(OFF);
			delay_ms(500);
		}
	}
	
	//ACC_SET_OFFSET();//���ٶȼ�ˮƽУ׼
	GYRO_SET_OFFSET();//��������ƫУ׼
	Data_Send_Offset(ACC_OFFSET.X,ACC_OFFSET.Y,ACC_OFFSET.Z,GYRO_OFFSET.X,GYRO_OFFSET.Y,GYRO_OFFSET.Z);//���ͼ��ٶȼ� ������ƫ��ֵ
}

void setup(void)// ��ʼ�����ú���
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�������ȼ����� 2
	LED_GPIO_Config();//����LED��IO��
	Delay_Init(72);//��ʱ������ʼ��
	USART1_Config(115200);//USART1��ʼ��
	I2C_INIT();
	delay_ms(1000);	
	MPU6050_INIT();//MPU6050��ʼ��
	MPU6050_INIT();
	PID_Init();//PID��ֵ��ʼ��
	Motor_Init(50);//PWM�����ʼ��
	RC_Init();//ң�������ճ�ʼ��(ң������ֵ����RC_CH[X]������)
	FlyControlAdjust();//�ж��Ƿ���Ҫ������Ӧ��У׼	
	
	//--------------------ִ�е��˴�˵���Ѿ�������--------------------
	Led_Refresh_Init();//LED��˸�жϿ���
	FlyControlInit();//�ɿ�ϵͳ��ʼ��(ң����ͨ����ʼ����ˮƽλ�ó�ʼ������������ƫ��ʼ��)
	
	System_Time_Init();
}

int main(void)
{
	setup();
	
	while(1)
	{
		//------------------------------------����1 DMA��������------------------------------------
		if(DMA_DATA_SEND_FLAG == 1)
		{		
			
			//START//////////////////�����ϴ� 4ms////////////////////////////////////////
			Data_Send_Attitude(acc,gyro,mag,-Q_ANGLE.ROLL,Q_ANGLE.PITCH,Q_ANGLE.YAW);//�ϴ���̬����
			Data_Send_Control(RC_CH,(moto1-1000)/10,(moto2-1000)/10,(moto3-1000)/10,(moto4-1000)/10,512);//�ϴ������ң�ء���ѹ����
			
			//END////////////////////////////////////////////////////////////////////////
			
			DMA_DATA_SEND_FLAG = 0;
		}
		

		
		//------------------------------------�жϲ��ı����˸�ķ�ʽ------------------------------------
		if(ARMED == 1)//�Ѿ�����
		{
			led_state = 0;//LED 1������+2�ο���
		}
		if(ARMED == 0)
		{
			led_state = 2;//LED 1������
		}
		
	}
}



