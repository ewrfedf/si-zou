#include "imu.h"
#include "mpu6050.h"
#include "math.h"

ANGLE Q_ANGLE;


//����ת��Ϊ���ٶ�(�Ƕ�ÿ��)
float number_to_dps(s16 number)
{
	float temp;
	temp = (float)number*Gyro_Gain;
	return temp;
}

//����ת��Ϊ���ٶ�(����ÿ��)
float number_to_dps1(s16 number)
{
	float temp;
	temp = (float)number*Gyro_GainR;
	return temp;
}

//����ת��Ϊ���ٶ�
float number_to_g(s16 number)
{
	float temp;
	temp = (float)number*Acc_Gain;
	return temp;
}

//ʹ�ü��ٶȼ����ݼ���ŷ����
void Get_Accel_Angle(s16 x,s16 y,s16 z,float* roll,float* pitch)
{
	float temp;
	
	temp = sqrtf((float)(y*y+z*z));
	temp = atan2((float)x,temp)*(180/3.14159265);
	*pitch = temp;
	
	temp = sqrtf((float)(x*x+z*z));
	temp = atan2((float)y,temp)*(180/3.14159265);
	*roll = temp;

}

//���ٶȼ� �Ƕ� ��Ȩƽ���˲�
float roll_data[4];
float pitch_data[4];
void ACC_Angle_Filter(float roll_in,float pitch_in,float *roll_out,float *pitch_out)
{
	u8 i;
	
	//��ӿ�ʼ
	for(i=0;i<3;i++)
	{
		roll_data[3-i] = roll_data[3-i-1];
		pitch_data[3-i] = pitch_data[3-i-1];
	}
	roll_data[0] = roll_in;
	pitch_data[0] = pitch_in;
	//������
	
	//��Ȩƽ���˲�
	*roll_out = (65*roll_data[0]+20*roll_data[1]+10*roll_data[2]+5*roll_data[3])/100;
	*pitch_out = (65*pitch_out[0]+20*pitch_out[1]+10*pitch_out[2]+5*pitch_out[3])/100;
	
}



//�����ںϼ���ŷ����
//���룺���ٶ�(��/��)�����ٶ�ԭʼ����
//�����Q_ANGLE
void IMUupdate(s16 gx, s16 gy, s16 gz, s16 ax, s16 ay, s16 az,s16 yaw_gyro_tar)
{
	static float x1=0,y1=0,z1=0,x2=0,y2=0,z2=0;
	static float roll_a=0,pitch_a=0;
	
	Get_Accel_Angle(ax,ay,az,&roll_a,&pitch_a);//�õ����ٶȼƵ�ŷ���Ƕ�
	
	
// 	//�����Ǽ���Ƕ�
// 	Q_ANGLE.PITCH = Q_ANGLE.PITCH + number_to_dps(gy) * DT;
// 	Q_ANGLE.ROLL = Q_ANGLE.ROLL + number_to_dps(gx) * DT;		
// 	Q_ANGLE.YAW =  Q_ANGLE.YAW + number_to_dps(gz) * DT;		
	
	/*
	//һ�׻����˲�
	Q_ANGLE.ROLL = (Q_ANGLE.ROLL+number_to_dps(gx)*DT)*FILTER_A +roll_a*(1-FILTER_A);

	Q_ANGLE.PITCH = (Q_ANGLE.PITCH+number_to_dps(gy)*DT)*FILTER_A + pitch_a*(1-FILTER_A);
	
	Q_ANGLE.YAW =  Q_ANGLE.YAW + number_to_dps(gz)*DT;
	
	*/
	
	///*
	//���׻����˲�
	x1 = (pitch_a - Q_ANGLE.PITCH)*FILTER_K*FILTER_K;
	y1 = y1+x1*DT;
	z1 = y1 + 2*FILTER_K*(pitch_a-Q_ANGLE.PITCH)+number_to_dps(gy);
	Q_ANGLE.PITCH = Q_ANGLE.PITCH + z1*DT;

	x2 = (roll_a - Q_ANGLE.ROLL)*FILTER_K*FILTER_K;
	y2 = y2+x2*DT;
	z2 = y2 + 2*FILTER_K*(roll_a-Q_ANGLE.ROLL)+number_to_dps(gx);
	Q_ANGLE.ROLL = Q_ANGLE.ROLL + z2*DT;
	
	if(yaw_gyro_tar>=-5 && yaw_gyro_tar<= 5)//YAWҡ����������
	{
		Q_ANGLE.YAW =  Q_ANGLE.YAW + number_to_dps(gz)*DT;
	}
	//���ҡ�˽��п��� ��YAW�ǶȲ�����
	
	//*/
	
}





#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                       // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period�������ڵ�һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az)
{

  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  //float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
	
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
	
  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  vx = 2*(q1q3 - q0q2);												//��Ԫ����xyz�ı�ʾ
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //�������������õ���־������
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //�������л���
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//�����PI�󲹳��������ǣ����������Ư��
  gy = gy + Kp*ey + eyInt;
  //gz = gz + Kp*ez+ ezInt;				   							//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
	
	
	
  //��Ԫ�ص�΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // ��������Ԫ��
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
	
	//��Ԫ��ת����ŷ����
  //Q_ANGLE.YAW =  0.002 * MPU6050_GYRO_LAST.Z*Gyro_Gain;  // yaw GYRO_I.Z;  atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3+1)* 57.3;
	Q_ANGLE.YAW = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3+1)* 57.3;//yaw
	Q_ANGLE.PITCH = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE.ROLL = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	
	
	if(Q_ANGLE.ROLL>90 || Q_ANGLE.ROLL<-90)
	{
		if(Q_ANGLE.PITCH>0) Q_ANGLE.PITCH = 180 - Q_ANGLE.PITCH;
		if(Q_ANGLE.PITCH<0) Q_ANGLE.PITCH = -(180 + Q_ANGLE.PITCH);
	}
	
}

void IMUupdate2(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float cr, cp, cy, sr, sp, sy, cpcy, spsy;
	
	//ŷ����ת��Ԫ��
	cr = cos(Q_ANGLE.ROLL/114.6f);
	cp = cos(Q_ANGLE.PITCH/114.6f);
	cy = cos(Q_ANGLE.YAW/114.6f);
	
	sr = sin(Q_ANGLE.ROLL/114.6f);
	sp = sin(Q_ANGLE.PITCH/114.6f);
	sy = sin(Q_ANGLE.YAW/114.6f);
	
	cpcy = cp * cy;
	spsy = sp * sy;
	
	q0 = cr * cpcy + sr * spsy;
	q1 = sr * cpcy - cr * spsy;
	q2 = cr * sp * cy + sr * cp * sy;
	q3 = cr * cp * sy - sr * sp * cy;
	
	
	
	
	//��Ԫ�ص�΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // ��������Ԫ��
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
	
	//��Ԫ��ת����ŷ����
  //Q_ANGLE.YAW =  0.002 * MPU6050_GYRO_LAST.Z*Gyro_Gain;  // yaw GYRO_I.Z;  atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3+1)* 57.3;
	Q_ANGLE.YAW = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3+1)* 57.3;//yaw
	Q_ANGLE.PITCH = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE.ROLL = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	
}
/*

void AHRS_GetQ( Quaternion *pNumQ )
{
  fp32 ErrX, ErrY, ErrZ;
  fp32 AccX, AccY, AccZ;
  fp32 GyrX, GyrY, GyrZ;
	fp32 Normalize;
  static fp32 exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	Gravity V;
	
	// ���ٶȹ�һ��
	Normalize = Q_rsqrt(squa(sensor.acc.averag.x)+ squa(sensor.acc.averag.y) +squa(sensor.acc.averag.z));
	AccX = sensor.acc.averag.x*Normalize;
  AccY = sensor.acc.averag.y*Normalize;
  AccZ = sensor.acc.averag.z*Normalize;

	// ��ȡ��������
	V = Quaternion_vectorGravity(&NumQ);
	
	// �������
 	ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
 	
 	exInt = exInt + ErrX * KiDef;
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;

  GyrX = sensor.gyro.averag.x * Gyro_Gr + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
  GyrY = sensor.gyro.averag.y * Gyro_Gr + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = sensor.gyro.averag.z * Gyro_Gr + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
	
	// һ�����������, ������Ԫ��
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	
	// ��Ԫ����һ��
	Quaternion_Normalize(&NumQ);
}*/





