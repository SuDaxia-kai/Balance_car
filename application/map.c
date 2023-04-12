#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "bsp_delay.h"
#include "scaner.h"
#include "bsp_imu.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "tim_it.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "bsp_buzzer.h"

struct Map_State map = {0,0};

//u8 route[100] = {C8, C4, N20, P7, N20, 0XFF};

//u8 route[100] = {C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,0XFF};

//u8 route[100] = {N9,B9,N7,P5,N7,0XFF};

u8 route [100] = {B1,N1,P1,N1,B2,N4,N5,N6,P4,N6,N5,N4,N3,P3,N3,N10,0XFF};    //����·��

//u8 route[100] = {P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};   //qqb

//u8 route[100] = {B9,N7,0XFF};

//u8 route[100] = {P7,N20,0XFF};


//����·��
//u8 route[100]={B1,N1,P1,N1,B2,N4,N3,P3,N3,
//				N4,N5,
//				N6,P4,N6,C1,C2,N13,P6,N13,N19,C6,B7,N22,C9,P8,
//				C9,N22,B6,N20,P7,
//				N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,/*B8,N9,N10,
//				N12,N13,C2,C1,N6,N5,N4,B3,N2,P2,*/0XFF};
				
u8 door1route[100]={N15,C5,B4,N18,N16,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,
				    C9,P8,C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

					
u8 door2route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};				

u8 door3route[100]={N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

u8 door4route[100]={N12,N13,P6,N13,N18,B5,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

	
NODESR nodesr;	//�����м����
					

//��ͼ��ʼ��
void mapInit()
{
	u8 i=0;
	nodesr.nowNode.nodenum = N2;		//��ʼ��   //N2
	nodesr.nowNode.angle = 0;		//��ʼ�Ƕ�   //0
	nodesr.nowNode.function = 1;	//��ʼ����   //1
	nodesr.nowNode.speed = 80;//60             //80
	nodesr.nowNode.step= 20;//30               //20
	nodesr.nowNode.flag = CLEFT|RIGHT_LINE;    //CLEFT|RIGHT_LINE
//	for(i=0;i<107;i++)				//�ѵ�ͼ������Ϣ������ת���ɱ�������
//		Node[i].step*=58.22;
//	for(i=0;i<107;i++)				//�ѵ�ͼ������Ϣ������ת���ɱ�������
//		Node[i].flag|=STOPTURN;
	for(i=0;i<118;i++)			//ȫ��ͼ�ٶȵ���
		Node[i].speed*=1.8;	
}


u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{//�õ����ڵ㵽���ڽ��ĵ�ַ
	unsigned char rest = ConnectionNum[nownode];
	unsigned char addr = Address[nownode];
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)
		{			
			return addr;
		}
		addr++;
	}
	return 0;
}

u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;
	static uint16_t mul2sing = 0, sing2mul = 0;
	
	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //����
	{
		//���6��������5��������
		if (Scaner.ledNum >= 5)
		{
			seed = 0X0001;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed <<= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//�Ұ��
	{
		if (Scaner.ledNum>=5)
		{
			seed = 0X8000;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed >>= 1;
			}
			lnum = 0;
		}
	}

	if ((nodesr.nowNode.flag & CLEFT) == CLEFT)//��ֲ�·
	{
		//�������ڶ���������������һ��������
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x0006) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCLEFT) == MCLEFT)//��ֲ�·
	{
		//��������һ����������
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x0001) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCRIGHT) == MCRIGHT)//��ֲ�·
	{
		//��������һ����������
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x8000) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)//�ҷֲ�·
	{
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x6000) )
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & AWHITE) == AWHITE)//ȫ��
	{
		 if((Scaner.ledNum>=10&&(Scaner.detail&0x1FF8)==0x1FF8))
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2SING) == MUL2SING)//���ֲ�·
	{
//		 if(((Scaner.ledNum>=4&&Scaner.ledNum<=8)&&(Scaner.detail&0x07E0)==0x07E0))//����
//		{
//			return 1;
//		}
		if (Scaner.lineNum > 1 || Scaner.ledNum >= 5)
			++mul2sing;
		
		if (mul2sing>=3 && Scaner.lineNum==1 && Scaner.ledNum<=3) //����Ŀ�ɶ���һ��
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2MUL) == MUL2MUL)  //����Ŀ�ɶ��������
	{
		if (Scaner.lineNum>1 || Scaner.ledNum>=5)
			++mul2sing;
		if (mul2sing>=3 && Scaner.lineNum==1)
			++sing2mul;
		if (sing2mul>=3 && (Scaner.lineNum>1 || Scaner.ledNum>=5))
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	return 0;
}

void Cross()
{			
	float num = 0;
	static u8 _flag=1;			//��㶯����־λ
	float turn;
	float turnspeed;
	static u8 cnt = 0;
	if(map.point == 0)
	{
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//��ȡ��һ���
	}
	if(_flag==1)//ѭ��
	{
		
		if(fabsf(motor_all.Distance) >= 0.7*nodesr.nowNode.step) 
		{
			_flag=0;
			if(((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ))
			{
				if ((Scaner.detail & 0X0180) == 0X0180)
				mpuZreset(imu.yaw, nodesr.nowNode.angle);     //��ȡ������Z
				//nodesr.nowNode.flag&=~0x80;
			}
			
			//��������ɾ��
			/*if(((nodesr.nowNode.flag&RESTPID)==RESTPID))
			{
				//motor_pid_clear();
				nodesr.nowNode.flag&=~0x80;    //10000000
			}*/
			
			
			//����1��ת��Ƕ�С��10��
			//����2����һ��·�ڲ���T����·��
			if( (fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<=10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<=10))
			{													 		
				motor_all.Cincrement = 2;	  //ԭ����1
				motor_all.Cspeed = 0.8*nodesr.nowNode.speed;
				
			}
			else
			{
				motor_all.Cincrement = 2.5;	//Ĭ�ϼ��ٶ� ԭ����1.5
				if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
				{
					motor_all.Cspeed = 30;
				}
				if(0.5*nodesr.nowNode.speed>40)
				{
					motor_all.Cspeed=80;// ԭ�� 40 
				}
				else
				{
					motor_all.Cspeed=nodesr.nowNode.speed;// ԭ����0.5    0.9��ǰ���
				}
			}
		}
		else
		{
			motor_all.Cincrement = 2.5;	//Ĭ�ϼ��ٶ�  ԭ����1.5
			motor_all.Cspeed=nodesr.nowNode.speed;// 0.8ԭ����
		}
	}
	else if(_flag==0)//�ж�·��
	{
			map_function(nodesr.nowNode.function);
		
		if(deal_arrive())	//�ж��Ƿ񵽴�·��
		{
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//����·��
			
			buzzer_on();
			if(nodesr.nowNode.nodenum == N8)
			{
				delay_ms(50);
			}
			if(nodesr.nowNode.nodenum == N13)
			{
				delay_ms(200);
			}
			else
			{
				delay_ms(100);
			}
			buzzer_off();
			
		}	
	}
	if((nodesr.flag&0x04)==0x04)//ת��
	{	
		nodesr.flag&=~0x04;		//	�������·�ڱ�־
		if(route[map.point-1] != 255)	 //route[i]==255����һ����·������
		{		  
		        
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10))//�����һ���Ƕ��뵱ǰ���Ƕ���ͬ������Ҫת������Ҫ����	
			{													 		
				_flag=1;
			}
			else 
			{	                
				if(nodesr.nowNode.flag&STOPTURN)    //STOPTURN��־λ����ԭ��ת
				{      
					num = motor_all.Distance;
					if(nodesr.nowNode.nodenum == N22 || nodesr.nowNode.nodenum == C6 )   //C9��λ��Ҳ�����ж�
					{
						while(motor_all.Distance-num < 6);  //ԭ����16
					}
					else if(nodesr.nowNode.nodenum == C9)
					{
						while(motor_all.Distance-num < 3);
					}
					else if(nodesr.nextNode.nodenum == B6)
					{
						while(motor_all.Distance-num < 12);
					}
					else if(nodesr.nowNode.nodenum == N8)
					{
						while(motor_all.Distance-num < 8); 
					}
					else if(nodesr.nextNode.nodenum == N4)
					{
						while(motor_all.Distance-num < 27); 
					}
					else if(nodesr.nowNode.nodenum == N3)
					{
						while(motor_all.Distance-num < 5); 
					}
					else if(nodesr.nowNode.nodenum == C1)
					{
						while(motor_all.Distance-num < 22);
					}
					else if(nodesr.nowNode.nodenum == N12)
					{
						while(motor_all.Distance-num < 9);
					}
					else
					{
						while(motor_all.Distance-num<5);	//������ʱ�����ڵȴ��������ĵ���·��,ԭ����15
					}
					angle.AngleT = nodesr.nextNode.angle;  //ת���ԽǶ�
					pid_mode_switch(is_Turn);	
					while(fabs(nodesr.nextNode.angle-getAngleZ())>2);
					CarBrake();
					delay_ms(20);
					motor_pid_clear();
					_flag=1;

					/***����*********/
//					Turn_Angle_Relative(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));	
//					while(fabs(angle.AngleT - getAngleZ())>3);
//					CarBrake();
//					delay_ms(10);
//					motor_pid_clear();
//					_flag=1;				
					/****************/
					
					
				}
				else		//����ת
				{	
					if(fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<60)
					{
						pid_mode_switch(is_Free);
						turn=need2turn(getAngleZ(),nodesr.nextNode.angle);
						turnspeed=82*(180-fabs(need2turn(getAngleZ(),nodesr.nextNode.angle)))/360;
						while(fabs(turn)>4)
						{
							turn=need2turn(getAngleZ(),nodesr.nextNode.angle);
							turnspeed=84*(180-fabs(need2turn(getAngleZ(),nodesr.nextNode.angle)))/360;
							AdCircle(turnspeed,turn/1.4);
						}
						_flag=1;	
					}			
					else
					{
						num=motor_all.Distance;
						while(motor_all.Distance-num<15);
						angle.AngleT = nodesr.nextNode.angle;  //ת���ԽǶ�
						pid_mode_switch(is_Turn);	
						while(fabs(nodesr.nextNode.angle-getAngleZ())>2);						
						_flag=1;		
					}
				}
			}
			pid_mode_switch(is_Line);	
			nodesr.nowNode=nodesr.nextNode;	
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//��ȡ��һ���	
			
			//·�̼�¼����
			motor_all.Distance = 0;
			motor_all.encoder_avg = 0;
		} 
		else
		{		//���·������
			
			motor_all.Cspeed=0;
			CarBrake();
			_flag=1;
			map.routetime+=1;
		}	
		
	}
		
	if(nodesr.flag&0x20)	//��������ǹص����
	{				
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//����һ�����и���
		nodesr.nextNode.function = DOOR;
		nodesr.flag&=~0x20;
	}
}
	

void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;		//Ѱ��
		case UpStage	  :Stage();break;//ƽ̨
		case BBridge  	:Barrier_Bridge(150,30);break;//���� ���ȣ��ٶ�
		case BHill	  	:Barrier_Hill(1);break;//¥��
		case LBHill     :Barrier_Hill(2); break;  //˫¥��
		case SM         :Sword_Mountain();break;//��ɽ
		case View		    :view();break;//���� ��ת
		case View1      :view1();break;//���� ֱ��
		case BACK       :back();break;
		case BSoutPole	:South_Pole(205);break;//�ϼ�
		case QQB		    :QQB_1();break;//���ΰ�
		case BLBS       :Barrier_WavedPlate(50);break;//�̼��ٰ� �ٶȣ�����
		case BLBL		    :Barrier_WavedPlate(110);break;//�����ٰ� �ٶȣ�����
		case DOOR	  	  :door();break;//����
		case BHM        :Barrier_HighMountain(); break;//�����
		case Scurve	  	:S_curve();  break;
		case IGNORE     :ignore_node(); break;  //���Ըýڵ�
		case UNDER      :undermou();break;
		default:break;		
	}
}


