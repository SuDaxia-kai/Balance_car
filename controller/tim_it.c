#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "ahrs.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "speed_ctrl.h"
#include "imu.h"
#include "Filter.h"
#include "sin_generate.h"

#define Speed_Bias_Up 10
#define Speed_Bias_Down 10

static uint32_t tim6_tick = 0;
uint32_t const *TIM6_tick = &tim6_tick;
volatile uint32_t TIME_ISR_CNT;


/****************************************************************************
* @brief  Timed interrupt serive function of TIM6 with 1ms interrupt time
* @param  htim pointer of TIM6
*****************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		++tim6_tick;
		/***************The frequency of Attitude updating is 200Hz**************/
		if (*TIM6_tick % 5 == 0)
		{
			TIME_ISR_CNT++;  			// 每过5ms，计数+1

			get_imu_data();
			ahrs_update();

			motor_SUM.measure = read_encoder(2) + read_encoder(3);
			
			int acc = 0;
			acc = Car_control_param.kp * Pitch + Car_control_param.kd * this_gyro.x;   // 姿态解算 by 马哥
			
			float Velocity = Velocity_PI(&motor_SUM, &Car_control_param);
			acc += Velocity;
			// Deadband of Motor
			if(acc > 0)
			{
				acc += 3020;
			}
			else
			{
				acc -= 3020;
			}

			printf("{info:%d,%f,%f,%f}\r\n",acc,Velocity,motor_SUM.integral,Pitch);	

			motor_set_pwm(1, acc);   // 负
			motor_set_pwm(2, -acc);   // 正
		}
			
}





