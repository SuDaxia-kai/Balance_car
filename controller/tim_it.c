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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim6))
    {
			++tim6_tick;
		
		//����10ms
		if (*TIM6_tick % 2 == 0)
		{
			// ÿ��10ms������+1
			TIME_ISR_CNT++;
		  // ������̬
			get_imu_data();
			ahrs_update();
			
			
			motor_A.measure = read_encoder(2);
			motor_B.measure = read_encoder(3);
			// 
//			printf("A���ٶ��� %d, B���ٶ��� %d \r\n", motor_A.measure, motor_B.measure);
			
			//�������ӱ�������ȡ��������������ֵ
			motor_all.encoder_avg = motor_A.measure + motor_B.measure;
			
			// ����������������������˵�ǰ��pitch��
			
		}
			
		motor_A.target = motor_all.Aspeed;
		motor_B.target = motor_all.Bspeed;

	
		
		incremental_PID(&motor_A, &motor_pid_param);
		incremental_PID(&motor_B, &motor_pid_param);
		
		motor_set_pwm(1, (int32_t)motor_A.output);
		motor_set_pwm(2, (int32_t)motor_B.output);
			
    }
}





