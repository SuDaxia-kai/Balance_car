#include "bsp_motor.h"
#include "main.h"
#include "tim.h"



/**********
 * @brief Enable Timer
*/
void motor_pwm_enable(void)
{
	// ENABLE PWM_CHANNELS OF TIM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	// ENABLE ENCODER
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/**
	*@brief Ouput signal to the motor
*/
void motor_set_pwm(uint8_t motor, int32_t pid_out)
{
	int32_t ccr = 0;
	
	if (pid_out >= 0)
	{
		if (pid_out > MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR1 = 0; TIM1->CCR4 = ccr;	break;
			case 2: TIM4->CCR3 = 0; TIM4->CCR4 = ccr; break;
			default: ; //TODO
		}
	}
	
	else if (pid_out < 0)
	{
		if (pid_out < -MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = -pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR4 = 0; TIM1->CCR1 = ccr;	break;
			case 2: TIM4->CCR4 = 0; TIM4->CCR3 = ccr; break;
			default: ; //TODO
		}
	}
}


/*********************************************
* @brief  Read encoder counts per unit time
* @param  TIMX ID of Timer
* @return Velocity of Motors
**********************************************/
int read_encoder(uint8_t TIMX)
{
	int Encoder_TIM;    
	switch(TIMX)
	{
		case 2:  
			Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT = 0; /*L0step-=Encoder_TIM;*/
			break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;   /*L1step-=Encoder_TIM;*/
			break;	
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}



