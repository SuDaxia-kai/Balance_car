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

void pid_mode_switch(uint8_t target_mode);

//���Ժ�����/////////////////////////////////////////////
void usmart_turn_angle(int set);
/////////////////////////////////////////////////////////

#endif


