#ifndef __TIM_IT_H
#define __TIM_IT_H

#include "sys.h"

enum PID_Mode {
	is_No = 0,  //�ر����в���
	is_Free,   //�����л�ǰ��״̬
	is_Line,   //ѭ��
	is_Turn,   //ת��
	is_Gyro   //��ƽ��
};

extern uint32_t const *TIM6_tick;
extern volatile uint32_t TIME_ISR_CNT;
void pid_mode_switch(uint8_t target_mode);


#endif


