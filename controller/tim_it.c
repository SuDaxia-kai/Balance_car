#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "ahrs.h"
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
		float F2 = Speed_low_filter(&record2, read_encoder(2));
		float F3 = Speed_low_filter(&record3, read_encoder(3));
		++tim6_tick;
		if (*TIM6_tick % 5 == 0)
		{
			// 每过5ms，计数+1
			TIME_ISR_CNT++;
		  // 以 200Hz 的频率更新姿态
			get_imu_data();
			ahrs_update();

			motor_A.measure = F2;
			motor_B.measure = F3;

//			printf("A的速度是 %d, B的速度是 %d \r\n", motor_A.measure, motor_B.measure);
//			printf("{Motor velocity:%f,%d}\r\n",F3,read_encoder(3));

			
			//两个轮子编码器读取的脉冲数量的总值
			motor_all.encoder_avg = motor_A.measure + motor_B.measure;
			
			// 根据脉冲总数来计算机器人当前的pitch角
			
//			motor_A.target = motor_all.Aspeed;
//			motor_B.target = motor_all.Bspeed;
//	
//		
		
			/* 
			直立环 负反馈 PD调节
			计算加速度 acc = kp * \theta + kd * \dot{\theta}
			根据simulink可得系数 kp = 200, kd = 8;
			*/
			int acc = 0;
			acc = 620 * (int)Pitch + 6 * this_gyro.x;
			// 后期测试时需要的变量
			int acc_temp = 0;
			acc_temp = 620 * (int)Pitch + 6 * this_gyro.x;

			/*
			速度环 正反馈 PI调节
			计算加速度 acc = -kp(kp_1 * error + ki1 * sum(e(k)))
			根据线性调参经验，系数关系为 kp_1 = ki_1 * 200
			*/
			motor_B.target = 0;
			motor_A.target = 0;
			incremental_PID(&motor_B, &motor_pid_param);
			incremental_PID(&motor_A, &motor_pid_param);
			int accA = acc - motor_A.output;
			int accB = acc - motor_B.output;
			
//			printf("{acc compare:%d,%d}\r\n",acc,acc_temp);

			motor_set_pwm(1, accB);
			motor_set_pwm(2, -accA);
			
//			motor_set_pwm(1, (int32_t)motor_A.output);
//			motor_set_pwm(2, (int32_t)motor_B.output);
		}
			
}





