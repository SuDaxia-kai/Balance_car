#ifndef __TIM_IT_H
#define __TIM_IT_H

#include "sys.h"

enum PID_Mode {
	is_No = 0,  //关闭所有操作
	is_Free,   //保留切换前的状态
	is_Line,   //循迹
	is_Turn,   //转弯
	is_Gyro   //自平衡
};

extern uint32_t const *TIM6_tick;
extern volatile uint32_t TIME_ISR_CNT;
void pid_mode_switch(uint8_t target_mode);


#endif


