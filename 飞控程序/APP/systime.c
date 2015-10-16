#include "systime.h"
#include "refreshled.h"
#include "led.h"
#include "maincom.h"

#include "mpu6050.h"
#include "imu.h"
#include "control.h"
#include "rc.h"

#include "chuanPID.h"

volatile u32 time2_tick;//Time5������
volatile u32 time2_led;

s16 acc[3],gyro[3],mag[3];

void System_Time_Init(void)
{
	MPU6050_READ();//��ȡ
	Get_Accel_Angle(MPU6050_ACC_LAST.X,MPU6050_ACC_LAST.Y,MPU6050_ACC_LAST.Z,&Q_ANGLE.ROLL,&Q_ANGLE.PITCH);//���ٶ�����ĽǶ�Ϊ��ʼ�Ƕ�
	TIM2_Config();
}

/*
 * ��������TIM2_Config
 * ����  ��TIM2���� NVIC�ж����� 2ms�ж�һ��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//����NVIC��TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//ͨ������ΪTIM2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//��Ӧ1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//��TIM2�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//д������
	
	//����TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//����TIM2ʱ��
  //TIM_DeInit(TIM2);//TIM2��ʼ��Ϊȱʡֵ

	TIM_TimeBaseStructure.TIM_Period=2000;//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);//ʱ��Ԥ��Ƶ�� 72M/72
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//������Ƶ TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//����TIM2
    
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//�������жϱ�־
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//��������ж�
	TIM_Cmd(TIM2,ENABLE);//����TIM2����
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE);//�ȹر�TIM2ʱ�� �ȴ�ʹ��
		
}

/*
 * ��������TIM2_Start
 * ����  ������TIM2
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM2_Start(void)
{
	time2_tick = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

/*
 * ��������TIM2_Stop
 * ����  ���ر�TIM2
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM2_Stop(void)
{
	TIM_Cmd(TIM2,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE);
}

//*****TIM2�жϺ���2ms*****//
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)//���TIM2����ж��Ƿ���
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		
		time2_tick++;
		time2_tick = time2_tick%1000;//2HZ
		
		//LED0(ON);
		//START////////////////////���ݶ�ȡ+��̬����+PID���� 1.4ms////////////////////////////
		
		MPU6050_READ();//��ȡ
	
		acc[0] = MPU6050_ACC_LAST.X;
		acc[1] = MPU6050_ACC_LAST.Y;
		acc[2] = MPU6050_ACC_LAST.Z;
		
		gyro[0] = MPU6050_GYRO_LAST.X - GYRO_OFFSET.X;
		gyro[1] = -(MPU6050_GYRO_LAST.Y - GYRO_OFFSET.Y);
		gyro[2] = MPU6050_GYRO_LAST.Z - GYRO_OFFSET.Z;
		if(gyro[2]>=-5 &&gyro[2]<=5) gyro[2] = 0;//Z��������,Ư�ƴ���
		
		//printf("%d,%d,%d",acc[0],acc[1],acc[2]);
		IMUupdate(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],Vaule_2_Gyro());//��ȫ��̬�ǽ���
		//IMUupdate1(number_to_dps1(gyro[0]),number_to_dps1(gyro[1]),number_to_dps1(gyro[2]),acc[0],acc[1],acc[2]);
		//IMUupdate2(number_to_dps1(gyro[0]),number_to_dps1(gyro[1]),number_to_dps1(gyro[2]),acc[0],acc[1],acc[2]);
		
		CONTROL(Q_ANGLE.ROLL,Q_ANGLE.PITCH,Q_ANGLE.YAW,Value_2_Thr(),Value_2_Roll(),Value_2_Pitch(),Vaule_2_Gyro());//�Ե������PID����
		
		//CONTROL1(Q_ANGLE.ROLL,Q_ANGLE.PITCH,Q_ANGLE.YAW,Value_2_Thr(),Value_2_Roll(),Value_2_Pitch(),Vaule_2_Gyro(), acc, gyro);
		
		//END///////////////////////////////////////////////////////////////////////
		//LED0(OFF);
		
		
		if(time2_tick%5 == 0)//10ms  --  100HZ����һ��
		{
			
		}
		
		if(time2_tick%25 == 0)//50ms	--	20HZ����һ��
		{
			DMA_DATA_SEND_FLAG = 1;//�ϴ�����
		}
		if(time2_tick%50 == 0)//100ms  --  10HZ����һ��
		{
			
		}
		if(time2_tick%500 == 0)//1000ms  --  1HZ����һ��
		{
			if(ARMED == 1)
			{
				Is_DisArmed(RC_CH[3-1],RC_CH[4-1]);//�Ƿ����
			}
			else
			{
				Is_Armed(RC_CH[3-1],RC_CH[4-1]);//�Ƿ����
			}
		}
		
		time2_led++;
		if(time2_led>=1000)  time2_led=time2_led-1000;
	//	time2_led = time2_led%1000;//2msһ������

		switch(led_state)
		{
				case 0:Led_Flash0();break;
				case 1:Led_Flash1();break;
				case 2:Led_Flash2();break;
				case 3:Led_Flash3();break;
		}
		
		
	}		
}










