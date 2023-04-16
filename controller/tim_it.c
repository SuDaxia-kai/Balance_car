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
		// �����˲�����֪���û� Wincent https://www.zhihu.com/people/wincent-84 ������
		// ���г���Ϊ10
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
			// ÿ��5ms������+1
			TIME_ISR_CNT++;
		  // �� 200Hz ��Ƶ�ʸ�����̬
			get_imu_data();
			ahrs_update();
		}
		//����10ms
		if (*TIM6_tick % 10 == 0)
		{

			motor_A.measure = read_encoder(2);
			motor_B.measure = read_encoder(3);

//			printf("A���ٶ��� %d, B���ٶ��� %d \r\n", motor_A.measure, motor_B.measure);
			
			//�������ӱ�������ȡ��������������ֵ
			motor_all.encoder_avg = motor_A.measure + motor_B.measure;
			
			// ����������������������˵�ǰ��pitch��
			
		}
			
		motor_A.target = motor_all.Aspeed;
		motor_B.target = motor_all.Bspeed;

	
		
		incremental_PID(&motor_A, &motor_pid_param);
		incremental_PID(&motor_B, &motor_pid_param);
		
		/* 
		ֱ����
		������ٶ� acc = kp * \theta + kd * \dot{\theta}
		����simulink�ɵ�ϵ�� kp = 200, kd = 8;
		*/
		int acc = 0;
		acc += 200 * (int)Pitch + 8 * this_gyro.x;
		if(Pitch > 60) // ��ʱ�����Ѿ�ʧ����
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





