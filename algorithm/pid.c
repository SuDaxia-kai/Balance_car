#include "pid.h"
#include "bsp_motor.h"
#include "stdio.h"


struct I_pid_obj motor_A = {0,0,0,0,0,0};
struct I_pid_obj motor_B = {0,0,0,0,0,0};
struct PID_param motor_pid_param;

struct P_pid_obj gyroT_pid = {0,0,0,0,0,0};
struct P_pid_obj gyroG_pid = {0,0,0,0,0,0};
struct PID_param gyroT_pid_param, gyroG_pid_param;

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
	
	motor->output += pid->kp*proportion + pid->ki*integral + pid->kd*differential;
	
	motor->last2_bias = motor->last_bias;
	motor->last_bias = motor->bias;
}

//位置式PID
//带抗积分饱和
//带微分项低通滤波
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid)
{
	float differential = 0;
	
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
	
	//微分项低通滤波
	differential = (obj->bias - obj->last_bias) * pid->differential_filterK + 
					(1 - pid->differential_filterK) * obj->last_differential;
	
	obj->output = pid->kp * obj->bias + pid->ki * obj->integral + pid->kd * differential;
	
	obj->last_bias = obj->bias;
	obj->last_differential = differential;
	
	return obj->output;
}


void pid_init(void)
{
	motor_pid_param.outputMax = MOTOR_PWM_MAX + 100;
	motor_pid_param.kp = -90;
	motor_pid_param.ki = -0.45;
	motor_pid_param.kd = 0;//25
	motor_pid_param.differential_filterK = 0.5;
	motor_pid_param.actualMax = 200;
	
//	gyroT_pid_param.kp = 1.0;
//	gyroT_pid_param.ki = 0.004;
//	gyroT_pid_param.kd = 0.5;
//	gyroT_pid_param.differential_filterK = 1;
//	gyroT_pid_param.outputMax = 80;
//	gyroT_pid_param.outputMin = -80;
//	
//	gyroG_pid_param.kp = 1.2;  //原来1.2
//	gyroG_pid_param.ki = 0.004;
//	gyroG_pid_param.kd = 0.5;
//	gyroG_pid_param.differential_filterK = 0.5;
//	gyroG_pid_param.outputMax = 100;
//	gyroG_pid_param.outputMin = -100;
	
	motor_pid_clear();
}


void motor_pid_clear(void)
{
	motor_A = (struct I_pid_obj){0,0,0,0,0,0};
	motor_B = (struct I_pid_obj){0,0,0,0,0,0};
}




