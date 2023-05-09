#include "pid.h"
#include "bsp_motor.h"
#include "ahrs.h"
#include "stdio.h"

struct P_pid_obj motor_SUM = {0,0,0,0,0,0};
struct PID_param Car_control_param;

/*********************************************************************** 
直立环 负反馈 PD调节 微分先行
计算加速度 acc = kp * e(k) + kd * \dot{\theta}
根据simulink可得系数 kp = 200, kd = 8;
************************************************************************/
float UpRight_PD(PidPtr pid, int Movement)
{
	return (pid->kp * (Pitch - Movement) + pid->kd * this_gyro.x);
}

/************************************************************************
 @brief PI positive feedback control of speed to accelerate the car
				back to the right. 		
根据线性调参经验，系数关系为 kp_1 = ki_1 * 200
************************************************************************/
float Velocity_PI(PPidPtr obj, PidPtr pid)
{ 
	obj->bias = obj->measure - obj->target;
	obj->encoder *= 0.7;
	obj->encoder += obj->bias*0.3;
	obj->integral += obj->encoder;
	if(obj->integral>10000)  obj->integral=10000;              
	if(obj->integral<-10000) obj->integral=-10000;              
	obj->output = pid->Velocity_kp * obj->encoder + pid->Velocity_ki * obj->integral;
	return obj->output;
}

/************************************************************************
速度环 正反馈 PI调节
计算加速度 acc = -kp(kp_1 * error + ki1 * sum(e(k)))
根据线性调参经验，系数关系为 kp_1 = ki_1 * 200
************************************************************************/
float Turn_D(float gyroz, PidPtr pid)
{
	return pid->Turn_kd * gyroz;
}

void pid_init(void)
{
	Car_control_param.outputMax = MOTOR_PWM_MAX + 100;
	Car_control_param.kp = 330;  // 420  330  200
	Car_control_param.kd = 20;   // 4   2.5    9
	Car_control_param.Velocity_kp = -3.8;
	Car_control_param.Velocity_ki = Car_control_param.Velocity_kp/185;
	Car_control_param.Turn_kd = 12;
	Car_control_param.differential_filterK = 0.5;
	Car_control_param.actualMax = 200;
	motor_pid_clear();
}


void motor_pid_clear(void)
{
	motor_SUM = (struct P_pid_obj){0,0,0,0,0,0,0};
}




