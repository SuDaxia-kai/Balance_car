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
			// ÿ��5ms������+1
			TIME_ISR_CNT++;
		  // �� 200Hz ��Ƶ�ʸ�����̬
			get_imu_data();
			ahrs_update();

			motor_A.measure = F2;
			motor_B.measure = F3;

//			printf("A���ٶ��� %d, B���ٶ��� %d \r\n", motor_A.measure, motor_B.measure);
//			printf("{Motor velocity:%f,%d}\r\n",F3,read_encoder(3));

			
			//�������ӱ�������ȡ��������������ֵ
			motor_all.encoder_avg = motor_A.measure + motor_B.measure;
			
			// ����������������������˵�ǰ��pitch��
			
//			motor_A.target = motor_all.Aspeed;
//			motor_B.target = motor_all.Bspeed;
//	
//		
		
			/* 
			ֱ���� ������ PD����
			������ٶ� acc = kp * \theta + kd * \dot{\theta}
			����simulink�ɵ�ϵ�� kp = 200, kd = 8;
			*/
			int acc = 0;
			acc = 620 * (int)Pitch + 6 * this_gyro.x;
			// ���ڲ���ʱ��Ҫ�ı���
			int acc_temp = 0;
			acc_temp = 620 * (int)Pitch + 6 * this_gyro.x;

			/*
			�ٶȻ� ������ PI����
			������ٶ� acc = -kp(kp_1 * error + ki1 * sum(e(k)))
			�������Ե��ξ��飬ϵ����ϵΪ kp_1 = ki_1 * 200
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





