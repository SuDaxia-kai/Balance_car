#include "barrier.h"
#include "sys.h"
#include "bsp_delay.h"
#include "speed_ctrl.h"
#include "bsp_motor.h"
#include "pid.h"
#include "bsp_imu.h"
#include "scaner.h"
#include "turn.h"
#include "map.h"
#include "tim_it.h"
#include "bsp_linefollower.h"
#include "pid.h"
#include "math.h"
#include "bsp_QRscan.h"
#include "QR_action.h"
#include "bsp_buzzer.h"
#include "scaner.h"
#include "bsp_led.h"
#include "stdio.h"

#define Up_pitch -15
#define Down_pitch 10
#define BACK_SPEED  -30
#define BACK_SPEED1 -10
#define GOSPEED 10

extern int BW_num[];
extern int value;

//������ϵ�к���
//1._shake��ƽ̨ʶ������У���ֹ��һЩ�������ص��³���ǰֹͣ,ͬʱ���ײ���Ŀ��
//2._Rshake�����ж����Ƿ񾭹����˰�
//3._pshakeʶ��������״̬
void Anti_shake(int Uneed_time)
{
	uint32_t i = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
	for(i = 0; i < Uneed_time ; i++)
	{
		HAL_Delay(1);
		while(Infrared_ahead == 0);
	}
}

int AI_shake(int Uneed_time)
{
	uint32_t i = 0, cnt = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
	for(i = 0; i < Uneed_time ; i++)
	{
		HAL_Delay(1);
		if(AI == 1)
		{
			cnt++;
		}
	}
	if(cnt >= Uneed_time*0.5)  return 1;
	return 0;
}

void Anti_encoder(int Uneed_time)
{
	uint32_t i = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
	for(i = 0; i < Uneed_time ; i++)
	{
		HAL_Delay(1);
		while(read_encoder(2) == 0);
	}
}

//void Anti_Rshake()
//{
//	uint32_t i = 0;
//	uint32_t cnt = 0;
//	while(i < 148)
//	{
//		for(i = 0; i < 150 ; i++)
//		{
//			HAL_Delay(1);
//			if(fabs(imu.roll) > 2) cnt++;
//			if(cnt == 30)
//			{
//				i = 0;
//				cnt = 0;
//				break;
//			}
//			//while(fabs(imu.roll) > 2);
//		}
//	}
//}

int Anti_Pshake()
{
	int sum = 0;
	for(uint32_t i = 0; i < 5; i++)
	{
		HAL_Delay(1);
		sum += imu.pitch;
	}
	if(sum/5 > 3) return 1;
	else return 0;
}

/************************************************/

void Stage()		//flag==1ʱȡ���ȽǶȣ�flag==0ʱȡ��ԽǶ�
{
	float num = 0; //distan = 0;
	
	pid_mode_switch(is_Line);
	
	if (nodesr.nowNode.nodenum == P2)
	{
		Stage_P2();
		return;
	}
	
	struct PID_param origin_param = line_pid_param;
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	while(imu.pitch > -5); //��б��ʱ��Ϊ����ƽ̨
	while(imu.pitch > -5);
	line_pid_param = origin_param;
//	angle.AngleG = getAngleZ();
	angle.AngleG = nodesr.nowNode.angle;
	motor_all.Gspeed = 50;
	pid_mode_switch(is_Gyro);
	
	//while(Infrared_ahead == 0);		//ײ����
	
	Anti_shake(200);  //ײ����

//	num = motor_all.Distance;				//ǰ��һ�ξ���
//	while(motor_all.Distance - num < 10);
	
	//delay_ms(300);
	CarBrake();
	
	//HAL_Delay(1000);
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	num = motor_all.Distance;				//����һ�ξ���
	pid_mode_switch(is_No);
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
	while (num-motor_all.Distance < 8); //ԭ���Ǻ���6
	CarBrake();
	
	buzzer_on();
	HAL_Delay(1500);
	buzzer_off();
	if(nodesr.nowNode.nodenum == P1)
	{
		if(AI_shake(800)) 
		{
			BW_num[0] = 3;
//			printf("%d\r\n",BW_num[0]);
		}
		else
		{
			BW_num[0] = 4;
//			printf("%d\r\n",BW_num[0]);
		}
	}
	else if(nodesr.nowNode.nodenum == P3 || nodesr.nowNode.nodenum == P4)
	{
		if(AI_shake(100)) 
		{
			BW_num[1] = 5;
		}
		else
		{
			BW_num[1] = 6;
		}
	}
	else if(nodesr.nowNode.nodenum == P6)
	{
		if(AI_shake(100)) 
		{
			BW_num[2] = 7;
		}
		else
		{
			BW_num[2] = 8;
		}
	}
	
	motor_pid_clear();
//	HAL_Delay(400);
	
	//myAction(); //�����˶���+Ѱ����� 
	
//	/*****************��ɨ������ά��******************/
// 	if((BW_num[0] == 0 && nodesr.nowNode.nodenum == P1)
//	  ||(BW_num[1] == 0 && nodesr.nowNode.nodenum == P3)
//	  ||(BW_num[1] == 0 && nodesr.nowNode.nodenum == P4)
//	  ||(BW_num[2] == 0 && nodesr.nowNode.nodenum == P5)
//	  ||(BW_num[2] == 0 && nodesr.nowNode.nodenum == P6)
//	)
//	{
//		num = motor_all.Distance;				//����һ�ξ���
//		motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
//		while (num-motor_all.Distance < 4);
//		CarBrake();
//		//motor_pid_clear();
//		Turn_Angle_Relative(-30);
//		while (fabs(angle.AngleT-getAngleZ())>2);
//		Turn_Angle_Relative(30);
//		while (fabs(angle.AngleT-getAngleZ())>2);
//		motor_pid_clear();
//	}
//	/***************************************************/
	
	/**************����****************/
	Shine_o_f();       //����һ��
	actions(5);     
	delay_ms(100);
	actions(3);
	delay_ms(100);
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT-getAngleZ())>2);
	
	if(check_BW(nodesr.nowNode.nodenum)) //��ⱦ��
		{
			actions(1);
			delay_ms(50);
			Turn_Angle360();
			actions(2);
//			delay_ms(50);
		}
	actions(6);
//	delay_ms(100);
	/**********************************/
	//Turn_Angle_Relative(179);		//ת>=180��
	motor_pid_clear();
	motor_all.Cspeed=50;
	motor_all.Cincrement = 1.0;
//	line_pid_param.kp = 9;
	pid_mode_switch(is_Line);
	while (imu.pitch < Down_pitch);	//����ʱ��Ϊ�뿪ƽ̨  
	nodesr.nowNode.function=0;	//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}


void Stage_P2()		//flag==1ʱȡ���ȽǶȣ�flag==0ʱȡ��ԽǶ�
{
	//float num = 0;
	while(imu.pitch > Up_pitch); //��б��ʱ��Ϊ����ƽ̨
	angle.AngleG=getAngleZ();
	motor_all.Gspeed=40;
	pid_mode_switch(is_Gyro);
	while (Infrared_ahead == 0);

	CarBrake();
	
	Turn_Angle_Relative(182);		//ת>=180�ȣ�
	while (fabs(angle.AngleT - getAngleZ())>5);
	motor_pid_clear();
	nodesr.nowNode.function=0;	//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/***************************************************
������
************************************************/

void Barrier_Bridge(float step,float speed)	//������
{
	float num = 0;
	
	motor_all.Cspeed = 80;    //ԭ��80   90
	pid_mode_switch(is_Line);
	while(imu.pitch > Up_pitch);	//����ƽ��	
	//is_Up = true;
	motor_all.Gspeed = 90;    //90 
	angle.AngleG = nodesr.nowNode.angle;
	pid_mode_switch(is_Gyro);
	
	struct PID_param origin_param = gyroG_pid_param;
	gyroG_pid_param.kp = 1.7;
	
	while(imu.pitch <= Up_pitch);	//����
	//is_Up = false;

	//motor_all.Gspeed = 110;   //ԭ��65   90
	num = motor_all.Distance;
	while(imu.pitch < Down_pitch)          
	{
		if (infrared.inside_left == 1||infrared.outside_left == 1)
			angle.AngleG = nodesr.nowNode.angle - 8; 
		else if (infrared.inside_right == 1||infrared.outside_right == 1)
			angle.AngleG = nodesr.nowNode.angle + 8; 
		else
			angle.AngleG = nodesr.nowNode.angle;
		
		if (motor_all.Distance-num < 85)
			motor_all.Gspeed = 90;     //ԭ����80  90
		else
			motor_all.Gspeed = 60;      //ԭ����65   75
	}
	angle.AngleG = nodesr.nowNode.angle;
	delay_ms(500);
	motor_all.Cspeed = 80;
	pid_mode_switch(is_Line);
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04;  //����·��
}

//��¥��
void Barrier_Hill(uint8_t order)  //¥������
{
	struct PID_param origin_param = line_pid_param;
	motor_all.Cspeed = 45;   //ԭ����60
	line_pid_param.kp = 50;
	pid_mode_switch(is_Line);
	while ( imu.pitch > -10 );  //ƽ��
	motor_all.Cspeed = 45;
	while(imu.pitch < -10);  //����
	while(imu.pitch<10);  //��һ��ƽ��
	
	if (order == 2)
	{
		while (imu.pitch > 10);  //�µ�һ����
		while (imu.pitch > -10); //����
		while (imu.pitch < -10);  //�ڶ�������
		while (imu.pitch < 10);  //�ڶ���ƽ��
	}
	line_pid_param = origin_param;
	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}



void Sword_Mountain()
{
	float num;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	num = motor_all.Distance;
	
	line_pid_param.kp = 50;
	
	motor_all.Cspeed = 20;   //ԭ��30
	
	//mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	while(motor_all.Distance - num < 50);
	angle.AngleG = getAngleZ();
	
	gyroG_pid_param.kp = 1.4;   //����ɽ����ƽ��kp
	motor_all.Gspeed = 20;
	pid_mode_switch(is_Gyro);

	while(imu.pitch > -2);
	buzzer_on();
	
	//motor_all.Gspeed = 20;  //30
	
//	motor_all.Lspeed = 24;
//	motor_all.Rspeed = 20;
//	pid_mode_switch(is_No);
	
	//angle.AngleG = 0;      //�˴���֪�����Ӧ��
	//pid_mode_switch(is_Gyro);
	while(imu.pitch < 2);
	gyroG_pid_param = origin_param1;
	line_pid_param = origin_param;
	buzzer_off();
	
	//while(Scaner.ledNum==0);
	 
	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/*****************************************************************************
����壬���������
*****************************************************************************/
void Barrier_HighMountain()
{
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	
	motor_all.Cspeed = 70; //�ʼ��û���µ��ٶ� 90
	motor_all.Cincrement = 2;
	pid_mode_switch(is_Line);
	while(imu.pitch>-17);
    
	motor_all.Cincrement = 1.8; //���µ��ٶ�  2
	motor_all.Cspeed = 60;//80
	
//	num = motor_all.Distance;
//	angle.AngleG = getAngleZ();
//	pid_mode_switch(is_Gyro);
//	motor_all.Gspeed = 40;
//	while(motor_all.Distance-num < 85);
//	
//	while (imu.pitch > -17);   //�ڵڶ���ƽ̨
//	while(Scaner.ledNum <= 10);
//	buzzer_on();
//	HAL_Delay(200);
//	buzzer_off();
//	pid_mode_switch(is_No);
//	motor_all.Lspeed = motor_all.Rspeed = -BACK_SPEED;
//	
//	while(Scaner.ledNum <= 10);
//	buzzer_on();
//	HAL_Delay(200);
//	buzzer_off();
//	motor_all.Cspeed = 60;
//	pid_mode_switch(is_Line);
////	num = motor_all.Distance;
////	while(motor_all.Distance-num < 85);
////	angle.AngleG = getAngleZ();
////	motor_all.Gspeed = 40;
////	pid_mode_switch(is_Gyro);

//	while(Scaner.ledNum <= 10);
//	buzzer_on();
//	HAL_Delay(200);
//	buzzer_off();
//	pid_mode_switch(is_No);
//	motor_all.Lspeed = motor_all.Rspeed = -BACK_SPEED;
	
	Anti_shake(300);

	CarBrake();
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);

	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

	
	while(num-motor_all.Distance < 8);
	CarBrake();
	
	motor_all.GyroT_speedMax = 60;   //����תȦ�ٶ�
  //Turn_Angle_Relative(181); //ת��
		
	Shine_o_f();       //����һ��
	actions(5);     //���㶯��
	delay_ms(100);
	actions(3);
//	delay_ms(100); 
	Turn_Angle_Relative(179);
	
	while(fabs(angle.AngleT - getAngleZ())>2); //�ж����
	if(check_BW(nodesr.nowNode.nodenum)) //��ⱦ��
		{
			actions(1);
			delay_ms(50);
			
			Turn_Angle_Relative(130);//160
			while(fabs(angle.AngleT - getAngleZ())>10);//4
			Turn_Angle_Relative(130);//160
			while(fabs(angle.AngleT - getAngleZ())>10);//4
			Turn_Angle_Relative(120);//120
			while(fabs(angle.AngleT - getAngleZ())>1);//4
			
			actions(2);
			delay_ms(50);
		}
//	actions(6);
//	delay_ms(100);
	
		
	motor_all.GyroT_speedMax = 80;   //�ָ�ԭ����ת���ٶ�
	
	motor_pid_clear();
	delay_ms(100);

	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	HAL_Delay(500);
	CarBrake();
	angle.AngleG = getAngleZ();
	
	Barrier_Down_HighMountain(60);

	line_pid_param = origin_param;

	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/*****************************************************************************
�������ܣ�����壬��ת��180�����
*****************************************************************************/
void Barrier_Down_HighMountain(float speed)
{
	
	struct PID_param origin_param1 = gyroG_pid_param;
	
	motor_all.Gspeed = 20;

	pid_mode_switch(is_Gyro);
	
	while(Scaner.ledNum <= 2)
	{
		buzzer_on();
	}
	
	while(Scaner.ledNum >= 10)
	{
		buzzer_off();
	}
	while(Scaner.ledNum >= 10);
	CarBrake();
	HAL_Delay(500);
	if(!(Scaner.detail & 0x180))
	{
		int weight_1 = Scaner.detail & 0x3f;
		int weight_2 = Scaner.detail & 0xfc00;
		if(weight_1)
		{
			pid_mode_switch(is_No);
			motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
			int num = motor_all.Distance;
			while(num - motor_all.Distance < 22);
			CarBrake();
			Turn_Angle_Relative(15);
			while (fabs(angle.AngleT-getAngleZ())>2);

			gyroG_pid_param.kp = 6;
			
			motor_all.Gspeed = 20;

			pid_mode_switch(is_Gyro);
		}
		else if(weight_2)
		{
			pid_mode_switch(is_No);
			motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
			int num = motor_all.Distance;
			while(num - motor_all.Distance< 22);
			CarBrake();
			Turn_Angle_Relative(-15);
			while (fabs(angle.AngleT-getAngleZ())>2);

			gyroG_pid_param.kp = 3;
			
			motor_all.Gspeed = 20;

			pid_mode_switch(is_Gyro);
		}
	}
	motor_all.Gspeed = 40;

	pid_mode_switch(is_Gyro);
	
	while(imu.pitch<20);
	gyroG_pid_param = origin_param1;
	actions(6);
	motor_all.Cspeed = speed;
	pid_mode_switch(is_Line);

	while(imu.pitch>5)
	{
		if (Scaner.ledNum >= 10)
		{
			angle.AngleG = getAngleZ();
			motor_all.Gspeed = speed;
			pid_mode_switch(is_Gyro);
//			buzzer_off();
		}
	}
	
	while(imu.pitch<15);
	
	pid_mode_switch(is_Line);
	motor_all.Cspeed = speed;
	while(imu.pitch>=10);
}


void view()//�򾰵�	
{	
	float num = 0;
	while(Infrared_ahead == 0);		//ײ����
	delay_ms(200);
	num=motor_all.Distance;
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(num-motor_all.Distance<3);
	CarBrake();
	delay_ms(100);
	Turn_Angle_Relative(181);
	while(fabs(angle.AngleT-getAngleZ())>5);
	motor_pid_clear();
	motor_all.Cspeed=30;
	pid_mode_switch(is_Line);
	delay_ms(200);
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}

void view1()//�򾰵�	
{	
	pid_mode_switch(is_Line);
	while(Infrared_ahead == 0);		//ײ����
	delay_ms(100);
	//mpuZreset(gyro.yaw,nodesr.nowNode.angle-10);
	CarBrake();
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}

void back()
{
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(infrared.outside_left == 0 && infrared.outside_right == 0);
	CarBrake();
	//ת���ԽǶ�
	angle.AngleT = nodesr.nextNode.angle;
	pid_mode_switch(is_Turn);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	//motor_all.Cspeed=15;
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}
/*****************************************************************************
�����˰�
�ٶ�   ��  ���˰峤�ȣ�cm��
������Ҫ�Ƿ�ֹ������ǰ���벨�˰壬Ȼ����ǰ�����������ʧ��
������һ�γ��ȣ���֤�ܽ��벨�˰壬�ұ�����ǰ���г��Ѿ���ȥ���˰塣
*****************************************************************************/
void Barrier_WavedPlate(float lenght)//���˰峤��
{
	float num = 0;
	//while(imu.pitch>-4);	
	//TIM8_PWM(2400,600,1500,1900);		//̧��Ѱ����
//	Turn_Angle(need2turn(getAngleZ(),nodesr.nowNode.angle),36);
//	while(fabs(Turn_Angle_Targe-getAngleZ())>5);

	motor_all.Cspeed = 40;   
	pid_mode_switch(is_Line);
	num = motor_all.Distance;
	while( motor_all.Distance-num < lenght);	
	//TIM8_PWM(2400,600,1500,2500);
	//while(Scaner.detail==0);
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��

}


void South_Pole(float length)
{
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	
	motor_all.Cspeed = 40;
	pid_mode_switch(is_Line);
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	
	while(imu.pitch>-20);
	motor_all.Cspeed = 95;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 55)
	
		angle.AngleG = getAngleZ();
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 70;

	//while(imu.pitch<-10);
	
	/*while(Infrared_ahead == 0);
	delay_ms(300);*/
	Anti_shake(250);
	CarBrake();
	
	//myAction(); //�����˶���+Ѱ�����
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);
	
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

	while(num - motor_all.Distance < 10);
	//Turn_Angle_Relative(182);
	CarBrake();
	
	Shine_o_f();       //����һ��
	actions(5);     //���㶯��
	delay_ms(100);
	actions(3);
//	delay_ms(100); 
	Turn_Angle_Relative(179);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	if(check_BW(nodesr.nowNode.nodenum)) //��ⱦ��
		{
			actions(1);
			delay_ms(50);
			Turn_Angle360();
			actions(2);
			delay_ms(50);
		}
	actions(6);
	delay_ms(100);
		
	motor_pid_clear();
	
	motor_all.Cspeed = 65;
	pid_mode_switch(is_Line);
	while(imu.pitch<10);       //��������
	while(imu.pitch>10);       //�ص�ƽ��
	
	motor_all.Cspeed = 38;
	num = motor_all.Distance;
	while(motor_all.Distance-num < length);	
	
	line_pid_param = origin_param;  //�ָ�ԭ����PID����
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}

#if 0
void QQB_1(void)
{
	float num = 0;
	CatchsensorNum=-3.5;			//�ı�ѭ�����ĵ㣬���ж�·�ں����Ļ���
	PIDMode=Catch;
	motor_all.Cspeed=20;
	while(Scaner.detail!=0);	//ѭ���������ΰ�
	PIDMode=Free;					
	L0speed=20;L1speed=20;R0speed=20;R1speed=20;
	while(imu.pitch>-15);		//��⵽��Ƕȣ����������ΰ�
	delay_ms(500);
	num=motor_all.Distance;
	PIDMode=GYRO;				//�ȳ����������ΰ�������ƽ��
	angle.AngleG=getAngleZ();
	motor_all.Gspeed=15;
	while(motor_all.Distance-num<700);			//���������߹̶�����
	CarBrake();					//ֹͣ
	while(imu.pitch<15);		//�ȴ����ΰ�����
	delay_ms(600);				//�ȴ����ΰ�ͣ��
	PIDMode=Free;
	L0speed=5;L1speed=5;R0speed=30;R1speed=30;
	delay_ms(200);
	CarBrake();	
	if(Scaner.detail==0)
	{
		Turn_Angle(30,30);			//��ʱ��ת40��
		while(fabs(Turn_Angle_Targe-getAngleZ())>5);
	}
	PIDMode=Catch;				//����ѭ��
	motor_all.Cspeed=20;
	nodesr.nowNode.function=0;
}
#endif

/*void QQB_1(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 30;  //�����ٽ���Ѱ�� 60
	
	scaner_set.CatchsensorNum = line_weight[7];   //�������Ȩֵ
	line_pid_param.kp = 45;    //����KP 45
	
	
	if(nodesr.nowNode.nodenum == B9)
	{	
		while(fabs(motor_all.Distance - num) < 28);//�ʵ�����
		pid_mode_switch(is_No);
		angle.AngleT = -92;//-89
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
		angle.AngleG = -92;//-89
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 20;
		while(imu.pitch > Up_pitch);//����ƽ��
		motor_all.Gspeed = 20;
		while(imu.pitch <= Up_pitch );
		
//		//start����
//		while(imu.pitch > -1);//�ȴ�����
//		buzzer_on();
//		HAL_Delay(200);
//		buzzer_off();
//		motor_all.Lspeed = 20;
//		motor_all.Rspeed = 25; 
//		//end�ջ�
		
		while(imu.pitch <= -3);
		CarBrake();
		while(imu.pitch <= Down_pitch);
		angle.AngleT = -25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
	}
	else if(nodesr.nowNode.nodenum == B8)
	{	
		while(fabs(motor_all.Distance - num) <32);
		pid_mode_switch(is_No);
		angle.AngleT = 90;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
		angle.AngleG = 90;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 30;
		while(imu.pitch > Up_pitch);//����ƽ��
		motor_all.Gspeed = 30;
		while(imu.pitch <= Up_pitch );
		
//		//start����
//		while(imu.pitch > -1);//�ȴ�����
//		motor_all.Lspeed = 18;
//		motor_all.Rspeed = 25; 
//		while(imu.pitch < -16);
//		//end�ջ�
		
		CarBrake();
		angle.AngleT = -25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
	}
	
	
//	pid_mode_switch(is_Line);
//	motor_all.Cspeed = 20;
	scaner_set.CatchsensorNum = 0;
	line_pid_param = origin_param;  //�ָ�ԭ����PID����
	nodesr.nowNode.function=0;
	nodesr.flag|=0X04;	//����·��
	
}*/

void QQB_1(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 60;  //�����ٽ���Ѱ��
	
	scaner_set.CatchsensorNum = line_weight[6];    //�������Ȩֵ
	line_pid_param.kp = 45;    //����KP 
	
	if(nodesr.nowNode.nodenum == B9)
	{	
		while(infrared.outside_right == 0);
		CarBrake();
		
		buzzer_on();
		
		//תȥ���ΰ������
		angle.AngleT = -90;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		/*angle.AngleT = 89;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);*/
		
		angle.AngleG = -90;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 20;
		while(imu.pitch > Up_pitch);//����ƽ��
		while(imu.pitch <= Up_pitch )
		{
			buzzer_off();
			if (infrared.outside_left == 1)
				angle.AngleG -= 4; 
			else if (infrared.outside_right == 1)
				angle.AngleG += 4; 
			else
				angle.AngleG = nodesr.nowNode.angle;
		}
		while(imu.pitch <= -2);
		HAL_Delay(800);
		buzzer_off();
		CarBrake();
		while(imu.pitch <= Down_pitch);
		motor_all.Gspeed = 20;
		HAL_Delay(800);
		angle.AngleT = -55;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		//while(imu.pitch <= Down_pitch);
		
		/*angle.AngleT = 25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);*/
		
	}
	else if(nodesr.nowNode.nodenum == B8)
	{	
		while(fabs(motor_all.Distance - num) <68);
		angle.AngleT = 90;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		angle.AngleG = 90;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 30;
		while(imu.pitch > Up_pitch);//����ƽ��
		motor_all.Gspeed = 30;
		while(imu.pitch <= Up_pitch );
		CarBrake();
//		angle.AngleT = -25;
//		pid_mode_switch(is_Turn);
//		while(fabs(angle.AngleT - getAngleZ())>5);
	}
	
	
//	pid_mode_switch(is_Line);
//	pid_mode_switch(is_Gyro);
//	num = motor_all.Distance;
//	motor_all.Gspeed = 10;
//	while(fabs(motor_all.Distance - num) < 5);
	scaner_set.CatchsensorNum = line_weight[8];    //�������Ȩֵ
	line_pid_param.kp = 45;    //����KP 
	motor_all.Cspeed = 60;  //�����ٽ���Ѱ��
	pid_mode_switch(is_Line);
	num = motor_all.Distance;
	while(fabs(motor_all.Distance - num) < 40);
	line_pid_param = origin_param;  //�ָ�ԭ����PID����
	scaner_set.CatchsensorNum = 0;
	nodesr.nowNode.function=0;
	nodesr.flag|=0X04;	//����·��
	
}

void door()
{
	float num = 0;
	//	buzzer_on();
	static u8 flag=0;
	motor_all.Cspeed = 60;
	pid_mode_switch(is_Line);
	if(flag==2)	
	{
		route_reset(flag);
		flag = 3;
		return;
	}
	else if(flag==4)
	{
		route_reset(flag);
		return;
	}
	while(1)
	{		//��һ�������ȫ��ֹͣ��
		if(Scaner.ledNum>=8)
		{
			buzzer_on();
			CarBrake();
			// ʶ����̵Ƶ�ʱ��
			HAL_Delay(400);    
			if(value == 1)
			{
				buzzer_off();
				if(flag==0)	//��һ��·�ڵĽ�ָͨʾ���Ǻ��ƣ������ܰ�·����������
				{
					map.point -= 2;
					route[map.point] = N8;
					pid_mode_switch(is_No);			//����
	
					//motor_L0.target = motor_L1.target = BACK_SPEED;
					//motor_R0.target = motor_R1.target = BACK_SPEED;
					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
					num = motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N10,N3)];		//��������nowNode
					nodesr.nowNode.step = 45;
					nodesr.nowNode.flag|=STOPTURN;
					nodesr.nowNode.speed=120;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;
					flag=1;
					return ;
				}
				else if(flag==1)//�ڶ���Ҳ�Ǻ�ɫ�������ܰ�·����������
				{		
					map.point-=2;		
					route[map.point]=N4;
					pid_mode_switch(is_No);			//����

					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

					num=motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N8,N3)];		//��������nowNode
					nodesr.nowNode.step = 30;
					nodesr.nowNode.speed = 120;
					nodesr.nowNode.flag|=STOPTURN;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;				
					flag=2;
					return ;
				}		
				else if(flag==3)
				{
					map.point-=2;		
					route[map.point]=N8;
					pid_mode_switch(is_No);	 //����
	
					//motor_L0.target = motor_L1.target = BACK_SPEED;
					//motor_R0.target = motor_R1.target = BACK_SPEED;
					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
					num=motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N12,N5)];		//��������nowNode
					nodesr.nowNode.step=48;
					nodesr.nowNode.speed=120;
					nodesr.nowNode.flag|=STOPTURN;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;				
					flag=4;
					return ;
				}
			}	
			else
			{
				// buzzer_off();
				motor_all.Cspeed=60;
				pid_mode_switch(is_Line);
				nodesr.nowNode.function=1;
				HAL_Delay(1000);
				break;
			}
		}
		else 
		{
			motor_all.Cspeed=60;
			pid_mode_switch(is_Line);
		}				
	}

	if(flag != 3)   route_reset(flag);//·����������

}


void route_reset(u8 flag)
{
	u8 temp=0,i=0;
	temp = map.point - 1;
	if(flag==0)//��һ���ſ���
	{			
		while(1)
		{	
			route[temp++]=door1route[i++];		//·������
			if(door1route[i]==255)
			{
				route[temp]=door1route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N10)];		//��������nowNode
				nodesr.nextNode = Node[getNextConnectNode(N10,route[map.point-1])];	//��������nextNode	
				nodesr.nowNode.step=20;
				nodesr.nowNode.speed=120;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
	else if(flag==1)//�ڶ����ſ���
	{		
		while(1)
		{	
			route[temp++]=door2route[i++];	//·������
			if(door2route[i]==255)
			{
				route[temp]=door2route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//��������nowNode
				nodesr.nextNode = Node[getNextConnectNode(N8,route[map.point-1])];	//��������nextNode	
				nodesr.nowNode.step=20;
				nodesr.nowNode.speed=120;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}	
	else if(flag==2)//ȥ�����ĸ�ͨ����
	{	
		while(1)
		{	
			route[temp++]=door3route[i++];	//·������
			if(door3route[i]==255)
			{
				route[temp]=door3route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N4)];		//��������nowNode
				nodesr.nextNode = Node[getNextConnectNode(N4,route[map.point-1])];	//��������nextNode
				nodesr.nowNode.step=20;
				nodesr.nowNode.speed=120;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
	else if(flag==4)//�������ſ���
	{
		while(1)
		{	
			route[temp++]=door4route[i++];	//·������
			if(door4route[i]==255)
			{
				route[temp]=door4route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//��������nowNode
				nodesr.nextNode = Node[getNextConnectNode(N8,route[map.point-1])];	//��������nextNode
				nodesr.nowNode.step=10;
				nodesr.nowNode.speed=120;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
}

void undermou(void)
{
	float num = 0;
	while(imu.pitch > -3);
	motor_all.Cspeed = 60;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 50);
	buzzer_on();
	if(nodesr.nowNode.nodenum == N14)
	{
		motor_all.Cspeed = 110;
		num = motor_all.Distance;
		while(motor_all.Distance - num < 100);
		motor_all.Cspeed = 60;
	}
	while(!deal_arrive());
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}
void S_curve(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	motor_all.Cspeed = nodesr.nowNode.speed;
	
	if (nodesr.nextNode.nodenum == N13)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 110);
		motor_all.Cspeed = 60;
		scaner_set.CatchsensorNum = line_weight[11];  //C1->C2
		line_pid_param.kp = 40;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	else if (nodesr.nextNode.nodenum == C1)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 60);
		scaner_set.CatchsensorNum = line_weight[5];
		line_pid_param.kp = 70;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	
	line_pid_param = origin_param;
	scaner_set.CatchsensorNum = 0;
	
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}


void ignore_node(void)
{
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}




