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
#include "kalman_filter.h"
#include "remote_ctrl.h"

#define R_car 104.0
#define Rad2Deg (180.0f / PI)
#define MID   1500
#define CH2_2_M 0.01f
#define CH1_2_M 0.04f

static uint32_t tim6_tick = 0;
uint32_t const *TIM6_tick = &tim6_tick;
volatile uint32_t TIME_ISR_CNT;


/****************************************************************************
* @brief  Timed interrupt serive function of TIM6 with 1ms interrupt time
* @param  htim pointer of TIM6
*****************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim6))
	{
		static int Wheeled_odometer = 0;
		static float _Yaw           = 0;
		int movement                = 0;
		int angle_vel               = 0;
		++tim6_tick;
		/***************Control frequency is 200Hz**************/
		if (*TIM6_tick % 5 == 0)
		{
			TIME_ISR_CNT++;  			// 每过5ms，计数+1
			get_imu_data();
			ahrs_update();
			
			int v2 = read_encoder(2); int v3 = read_encoder(3);
			motor_SUM.measure = v2 + v3;
			Wheeled_odometer += (v3 - v2)/(PSC*REDUCTION_RATIO);
			float yaw_observation = Rad2Deg * ((Wheeled_odometer*PI*RADIUS_WHEEL) / (500*R_car));
			
			kalman_filter(yaw_observation
									 ,&_Yaw
									 ,&Q
									 ,R
			             ,&cov
			             ,0.005);
//			printf("{yaw_observation:%f,%f,%f}\r\n",_Yaw,Yaw,yaw_observation);

			int acc = 0;
			int lock = ppm_receiver.databuf[2];
			// 只有CH3是1500时才解锁
			if(lock < 1600 && lock > 1400)
			{
				movement = (ppm_receiver.databuf[1] - MID)*CH2_2_M;
				angle_vel = ((ppm_receiver.databuf[0] - MID)*CH1_2_M);
			}
			else
			{
				movement = 0;
				angle_vel = 0;
			}
			// 三环
			acc = UpRight_PD(&Car_control_param,movement);
			float Velocity = Velocity_PI(&motor_SUM, &Car_control_param);
			int BIAS_PWM = Turn_D(this_gyro.z+angle_vel, &Car_control_param);
			acc += Velocity;
			
			int accA = -acc + BIAS_PWM;
			int accB = acc + BIAS_PWM;


//			printf("{info:%d,%f,%f,%f}\r\n",acc,Velocity,motor_SUM.integral,Pitch);	
//			printf("{info:%f}\r\n", Pitch);	
//				printf("{remote:%d,%d}\r\n",ppm_receiver.databuf[0],ppm_receiver.databuf[1]);
//			printf("{value:%d}\r\n",capture_val);
//			printf("{remote_value:%f,%f}\r\n",((ppm_receiver.databuf[0] - MID)*CH1_2_M),((ppm_receiver.databuf[1] - MID)*CH2_2_M));

				
			motor_set_pwm(1, accB);    // 负
			motor_set_pwm(2, accA);    // 正

		}
	}
}







