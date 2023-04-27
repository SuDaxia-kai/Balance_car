#ifndef __PID_H
#define __PID_H

#include "sys.h"

struct P_pid_obj {
	float output;
	float bias;
	float measure;
	float encoder;
	float integral;
	float last_differential;
	float target;
};
typedef struct P_pid_obj* PPidPtr;

//differential_filterK: 微分项滤波系数，取值范围(0,1]
//系数越小滤波效应越大，当系数为1时不进行滤波
struct PID_param {
	float kp;
	float kd;
	float Velocity_kp;
	float Velocity_ki;
	float Turn_kd;
	float differential_filterK;
	float outputMin;
	float outputMax;
	float actualMax;
};
typedef struct PID_param* PidPtr;

extern struct P_pid_obj motor_SUM;
extern struct PID_param Car_control_param;

float Velocity_PI(PPidPtr obj, PidPtr pid);
float Turn_D(float gyroz, PidPtr pid);
void pid_init(void);
void motor_pid_clear(void);

void usmart_pid(uint16_t val,int deno,int mode);


#endif
