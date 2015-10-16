#include "CONTROL.h"   
#include "motor.h"
#include <math.h>
#include "chuanPID.h"

/*********************************/
float Pitch_i,Roll_i,Yaw_i;                                   //积分项
float Pitch_old,Roll_old,Yaw_old;                 //角度保存
float Pitch_d,Roll_d,Yaw_d;          //微分项
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//外环总输出
        //外环PID参数
float Pitch_shell_kp=280;//30 140
float Pitch_shell_kd=0;//
float Pitch_shell_ki=0;//
/*********************************/
float Roll_shell_kp=250;//30
float Roll_shell_kd=0;//10                 
float Roll_shell_ki=0;//0.08
/*********************************/
float Yaw_shell_kp=10;//10;//30
float Yaw_shell_kd=0;//10                 
float Yaw_shell_ki=0;//0.08;//0.08
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//陀螺仪保存
float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_kd_out;//内环单项输出
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出       
       
//内环PID参数
float Pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;

float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;

float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;



float Get_MxMi1(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}

void CONTROL1(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar,s16* ACC,s16* GYRO)
{
       
        ////////////////////////外环角度环(PID)///////////////////////////////
  Pitch_i+=(pit_now-pit_tar);
//-------------Pitch积分限幅----------------//
  if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
//-------------Pitch微分--------------------//
  Pitch_d=pit_now-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(pit_now-pit_tar) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//角度保存
  Pitch_old=pit_now;
/*********************************************************/       
       
        Roll_i+=(rol_now-rol_tar);
//-------------Roll积分限幅----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll微分--------------------//
  Roll_d=rol_now-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(rol_now-rol_tar) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll角度保存------------------//
  Roll_old=rol_now;
       
//-------------Yaw微分--------------------//
  Yaw_d=GYRO[2]-Yaw_old;
//-------------Roll  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(GYRO[2]-yaw_gyro_tar) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll角度保存------------------//
  Yaw_old=GYRO[2];
       
       
        ////////////////////////内环角速度环(PD)///////////////////////////////       
  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + GYRO[1]* 3.5);
  pitch_core_kd_out = Pitch_core_kd * (GYRO[1]  - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + GYRO[0] *3.5);
  Roll_core_kd_out  = Roll_core_kd  * (GYRO[0]  - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + GYRO[2] * 1);
  Yaw_core_kd_out  = Yaw_core_kd  * (GYRO[2]  - Gyro_radian_old_z);
       
       
  Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
  Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

  Gyro_radian_old_y = GYRO[1];
  Gyro_radian_old_x = GYRO[0];
  Gyro_radian_old_z = GYRO[2];   //储存历史值
       
//--------------------将输出值融合到四个电机--------------------------------//

    if(throttle>=1100)//对油门进行进行判断<1100的油门电机不转动！！！！！(为了安全很重要！！！)
		{
			moto1 = throttle + Roll_core_out - Pitch_core_out + Yaw_core_out;  moto1 = Get_MxMi1(moto1,2000,1000);
			moto2 = throttle - Roll_core_out - Pitch_core_out - Yaw_core_out;  moto2 = Get_MxMi1(moto2,2000,1000);
			moto3 = throttle - Roll_core_out + Pitch_core_out + Yaw_core_out;  moto3 = Get_MxMi1(moto3,2000,1000);
			moto4 = throttle + Roll_core_out + Pitch_core_out - Yaw_core_out;  moto4 = Get_MxMi1(moto4,2000,1000);
			   
			Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
		}
		else
		{
			moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
			Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
		}

}
