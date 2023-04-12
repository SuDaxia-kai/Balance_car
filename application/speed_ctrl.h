#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

struct Gradual
{
	float Last;			//�ı�ʱ����һ�ε�ֵ
	float Now;			//��ǰֵ
	float D_value;		//�ı�ʱ�̵�Ŀ��ֵ�뵱ǰֵ�Ĳ�
};

struct Motors
{
	float Aspeed,Bspeed,Cspeed;	//�ٶ�
	float Gspeed;					//��ƽ���ٶ�
	float GyroT_speedMax;	//ת������ٶ�
	float GyroG_speedMax;	//��ƽ������ٶ�
	
	int encoder_avg;	//���������� 
	float Distance;		//·��
	
	float Cincrement;	//ѭ�����ٶ�
	float Gincrement;	//��ѭ�����ٶ�  
	
	bool is_UP;
	bool is_DOWM;
};

extern volatile struct Motors motor_all;

extern struct Gradual TC_speed, TG_speed;

void gradual_cal(struct Gradual *gradual, float target, float increment);
void CarBrake(void);

#endif
