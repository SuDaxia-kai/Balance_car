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
			// 每过5ms，计数+1
			TIME_ISR_CNT++;

			get_imu_data();
			ahrs_update();

			motor_A.measure = read_encoder(2);
			motor_B.measure = read_encoder(3);
			
			/*********************************************************************** 
			直立环 负反馈 PD调节
			计算加速度 acc = kp * \theta + kd * \dot{\theta}
			根据simulink可得系数 kp = 200, kd = 8;
			************************************************************************/
			int acc = 0;
			acc = Car_control_param.kp * Pitch + Car_control_param.kd * this_gyro.x;   // 姿态解算 by 马哥
//			acc = Car_control_param.kp * imu.roll + Car_control_param.kd * imu.gyrox;    // 姿态解算 by WIT
//			printf("%d \r\n", acc);

			/************************************************************************
			速度环 正反馈 PI调节
			计算加速度 acc = -kp(kp_1 * error + ki1 * sum(e(k)))
			根据线性调参经验，系数关系为 kp_1 = ki_1 * 200
			************************************************************************/
			static float Velocity=0,Encoder_new=0,Encoder=0;
			static float Encoder_Integral=0;

			float Velocity_Kp=-0.75,Velocity_Ki=Velocity_Kp/100.0;//-185 -0.925
//			float Velocity_Kp=0,Velocity_Ki=Velocity_Kp/200.0;//-185 -0.925
			Encoder_new =(motor_A.measure + motor_B.measure)-0;                 
			Encoder *= 0.7;                                                      
			Encoder += Encoder_new*0.3;                                        
			Encoder_Integral +=Encoder;                                                            
			if(Encoder_Integral>380000)  Encoder_Integral=380000;              
			if(Encoder_Integral<-380000) Encoder_Integral=-380000;              
			Velocity = Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;    
			acc += Velocity;
//			motor_T.target = 0;
//			int Velocity = positional_PID(&motor_T, &Car_control_param);
			if(acc > 0)
			{
				acc += 3020;
			}
			else
			{
				acc -= 3020;
			}

			printf("{info:%d,%f,%f,%f}\r\n",acc,Velocity,Encoder_Integral,Pitch);	
//			printf("{angle:%f}\r\n",Pitch);	

			motor_set_pwm(1, acc);   // 负
			motor_set_pwm(2, -acc);   // 正
		}
			
}





