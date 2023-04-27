#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "ahrs.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "imu.h"
#include "Filter.h"

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
		/***************Control frequency is 200Hz**************/
		if (*TIM6_tick % 5 == 0)
		{
			TIME_ISR_CNT++;  			// 每过5ms，计数+1
			get_imu_data();
			ahrs_update();

			motor_SUM.measure = read_encoder(2) + read_encoder(3);
			
			int acc = 0;
			acc = Car_control_param.kp * Pitch + Car_control_param.kd * this_gyro.x;
			
			float Velocity = Velocity_PI(&motor_SUM, &Car_control_param);
			int BIAS_PWM = Turn_D(this_gyro.z, &Car_control_param);
			acc += Velocity;
			
			int accA = -acc + BIAS_PWM;
			int accB = acc + BIAS_PWM;


//			printf("{info:%d,%f,%f,%f}\r\n",acc,Velocity,motor_SUM.integral,Pitch);	
//			printf("{info:%f}\r\n", Pitch);	

			motor_set_pwm(1, accB);    // 负
			motor_set_pwm(2, accA);   // 正
		}
			
}





