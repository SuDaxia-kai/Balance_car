#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "sys.h"
#include "Filter.h"

#define MOTOR_PWM_MAX 6000

extern Moving_Filter record2;
extern Moving_Filter record3;
//ʹ�ܶ�ʱ��PWMͨ��
void motor_pwm_enable(void);

//���PWM���ú���
//motor: ������
//pid_out: ��ʱ��CCRֵ�����Ϊ7200���������޷�MOTOR_PWM_MAX����ֵΪ��ת����ֵΪ��ת
void motor_set_pwm(uint8_t motor, int32_t pid_out);

int read_encoder(uint8_t TIMX);

#endif

