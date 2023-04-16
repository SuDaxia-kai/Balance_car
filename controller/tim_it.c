#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "ahrs.h"
#include "bsp_motor.h"
#include "speed_ctrl.h"
#include "imu.h"
#include "sin_generate.h"

#define PI  3.1415926535

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
		// 滑动滤波，受知乎用户 Wincent https://www.zhihu.com/people/wincent-84 的启发
		// 队列长度为10
		static int s_i_encoderofRightMotor = 0;
		static int s_i_encoderofLeftMotor  = 0;
		static int s_i_headofERM           = 0;
		static int s_i_headofELM           = 0;
    if (htim == (&htim6))
    {
			s_i_encoderofRightMotor = s_i_encoderofRightMotor - s_i_headofERM/10 + read_encoder(2)/10;
			s_i_encoderofLeftMotor  = s_i_encoderofLeftMotor  - s_i_headofELM/10 + read_encoder(3)/10;
		if (*TIM6_tick % 5 == 0)
		{
			++tim6_tick;
			// 每过5ms，计数+1
			TIME_ISR_CNT++;
		  // 以 200Hz 的频率更新姿态
			get_imu_data();
			ahrs_update();
		}
		//周期10ms
		if (*TIM6_tick % 10 == 0)
		{

			motor_A.measure = read_encoder(2);
			motor_B.measure = read_encoder(3);

//			printf("A的速度是 %d, B的速度是 %d \r\n", motor_A.measure, motor_B.measure);
			
			//两个轮子编码器读取的脉冲数量的总值
			motor_all.encoder_avg = motor_A.measure + motor_B.measure;
			
			// 根据脉冲总数来计算机器人当前的pitch角
			
		}
			
		motor_A.target = motor_all.Aspeed;
		motor_B.target = motor_all.Bspeed;

	
		
		incremental_PID(&motor_A, &motor_pid_param);
		incremental_PID(&motor_B, &motor_pid_param);
		
		/* 
		直立环
		计算加速度 acc = kp * \theta + kd * \dot{\theta}
		根据simulink可得系数 kp = 200, kd = 8;
		*/
		int acc = 0;
		acc += 200 * (int)Pitch + 8 * this_gyro.x;
		if(Pitch > 60) // 此时机体已经失控了
		{
				motor_set_pwm(1, 0);
				motor_set_pwm(2, 0);
		}
		else
		{
				motor_set_pwm(1, acc);
				motor_set_pwm(2, -acc);
		}

//		motor_set_pwm(1, (int32_t)motor_A.output);
//		motor_set_pwm(2, (int32_t)motor_B.output);
			
    }
}





