#include "pid.h"
#include "bsp_motor.h"
#include "stdio.h"


struct I_pid_obj motor_A = {0,0,0,0,0,0};
struct I_pid_obj motor_B = {0,0,0,0,0,0};
struct P_pid_obj motor_T = {0,0,0,0,0,0};
struct PID_param Car_control_param;

/**
*@brief 
*/
void incremental_PID (IPidPtr motor, PidPtr pid)
{
	float proportion = 0, integral = 0, differential = 0;
	
	motor->bias = motor->target - motor->measure;
	
	proportion = motor->bias - motor->last_bias;
	
	//抗积分饱和
	if (motor->output > pid->outputMax || motor->measure > pid->actualMax)
	{
		if (motor->bias < 0)
			integral = motor->bias;
	}
	else if (motor->output < -pid->outputMax || motor->measure < -pid->actualMax)
	{
		if (motor->bias > 0)
			integral = motor->bias;
	}
	else
	{
		integral = motor->bias;
	}
	
	differential = (motor->bias - 2 * motor->last_bias + motor->last2_bias);
	
	motor->output += pid->Velocity_kp*proportion + pid->Velocity_ki*integral + pid->Velocity_kd*differential;
	
	motor->last2_bias = motor->last_bias;
	motor->last_bias = motor->bias;
}

//位置式PID
//带抗积分饱和
//带微分项低通滤波
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid)
{
	float differential = 0;
	static int last_integral = 0;
	
	obj->bias = obj->target - obj->measure;
	
	// 积分遇限削弱法
	if (obj->output >= pid->outputMax)
	{
		if (obj->bias < 0)
			obj->integral += obj->bias;
	}
	else if (obj->output <= pid->outputMin)
	{
		if (obj->bias > 0)
			obj->integral += obj->bias;
	}
	else
	{
		obj->integral += obj->bias;
	}
	obj->integral = obj->integral*0.3 + last_integral*0.7;
	
	//微分项低通滤波
	differential = (obj->bias - obj->last_bias) * pid->differential_filterK + 
					(1 - pid->differential_filterK) * obj->last_differential;
	
	obj->output = pid->Velocity_kp * obj->bias + pid->Velocity_ki * obj->integral + pid->Velocity_kd * differential;
	
	obj->last_bias = obj->bias;
	obj->last_differential = differential;
	
	last_integral = obj->integral;
	
	return obj->output;
}


void pid_init(void)
{
	Car_control_param.outputMax = MOTOR_PWM_MAX + 100;
	Car_control_param.kp = 198;  // 420  330
	Car_control_param.kd = 6;   // 4   2.5
	Car_control_param.Velocity_kp = 0;
	Car_control_param.Velocity_ki = Car_control_param.Velocity_kp/200;
	Car_control_param.differential_filterK = 0.5;
	Car_control_param.actualMax = 200;
	motor_pid_clear();
}


void motor_pid_clear(void)
{
	motor_A = (struct I_pid_obj){0,0,0,0,0,0};
	motor_B = (struct I_pid_obj){0,0,0,0,0,0};
	motor_T = (struct P_pid_obj){0,0,0,0,0,0};
}




