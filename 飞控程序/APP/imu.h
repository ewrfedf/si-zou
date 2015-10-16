#ifndef __IMU_H_
#define __IMU_H_

#include "stm32f10x.h"

#define Gyro_Gain 		2000/32767;		//���������̣�+-2000��ÿ��
#define Gyro_GainR 		0.0010653;		//���������̣�+-2000����ÿ��
#define Acc_Gain 			2/32767;			//���ٶȼ����̣�+-2g

#define DT 	0.002f					//ÿ���˲���ʱ��Ƭ
#define FILTER_A  0.9983f //һ�׻����˲����˲�ϵ��
#define FILTER_K  0.45f//���׻����˲����˲�ϵ��

//����ϵͳ��ŷ����
typedef struct
{
	float ROLL;
	float PITCH;
	float YAW;
}ANGLE;


typedef __IO struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef __IO struct {
  float x;
  float y;
  float z;
} Gravity;

typedef float          fp32; 
extern ANGLE Q_ANGLE;

float number_to_dps(s16 number);
float number_to_dps1(s16 number);
float number_to_g(s16 number);

void Get_Accel_Angle(s16 x,s16 y,s16 z,float* roll,float* pitch);//ʹ�ü��ٶȼ����ݼ���ŷ����
void IMUupdate(s16 gx, s16 gy, s16 gz, s16 ax, s16 ay, s16 az,s16 yaw_gyro_tar);//�����ںϼ���ŷ����

void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az);//������ ���ٶȼ�  ��Ԫ����̬����
void IMUupdate2(float gx, float gy, float gz, float ax, float ay, float az);//������ ��Ԫ��������̬����







#endif


