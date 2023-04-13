#include "speed_ctrl.h"
#include "tim_it.h"
#include "pid.h"

volatile struct Motors motor_all = {
	.Aspeed = 0,
	.Bspeed = 0,
	.encoder_avg = 0,
	.is_UP = false,
	.is_DOWM = false,
};

struct Gradual TC_speed = {0,0,0}, TG_speed = {0,0,0};


void CarBrake(void)
{
	motor_all.Aspeed = motor_all.Bspeed = motor_all.Cspeed = 0;
}

//以一次函数缓慢加速或者缓慢停止
void gradual_cal(struct Gradual *gradual, float target, float increment)
{
	uint8_t direction = 0;
	
	if(target - gradual->Now < 0)
		direction = 0;
	else
		direction = 1;
	
	if(gradual->Now != target)
	{
		if (direction)
			gradual->Now += increment;
		else
			gradual->Now -= increment;
	}
	else 
	{	
		return;
	}
	
	if(direction == 1)
	{
		if(gradual->Now > target)
		{
			gradual->Now = target;
		}
	}
	else if(direction == 0)
	{
		if(gradual->Now < target)
		{
			gradual->Now = target;
		}
	}
}

