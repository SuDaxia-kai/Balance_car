#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "speed_ctrl.h"

#define LINE_SPEED_MAX 50

#define Speed_Compensate

const float line_weight[16] = {-3.1,-2.4,-1.8,-1.3,-0.9,-0.6,-0.4,-0.3,0.3,0.4,0.6,0.9,1.3,1.8,2.4,3.1};//0.3

struct Scaner_Set scaner_set = {0,0};

//float CatchsensorNum = 0;	//Ѱ��ʱ��Ŀ�괫����λ�ã�һ�������Ϊ0��΢΢���õ�
SCANER Scaner;
/*****************************************************************************
������д�˼��������ڣ���������07.16
�������ܣ�Ѱ�����ڶ�ʱ��֮�����У�����PIDMode��־λ�����ر�
���룺Ѱ���ٶ�
�������
*****************************************************************************/
void Go_Line(float speed)
{								//Ѱ������ڱ���Ѱ���ٶ�
	float Fspeed;				//����PID�����Ľ��
					
	line_pid_obj.measure = Scaner.pow;  //��ǰѭ�������ڵ�λ�ã�������-7��0��0��7
	line_pid_obj.target = scaner_set.CatchsensorNum;
	
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //����λ��PID����
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX)
		Fspeed = -LINE_SPEED_MAX;
	
	Fspeed *= fabsf(speed)/50;
	
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
}


void powCal(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore) //����ѭ�������Ƶ�λ��
{
	float _pow = 0;
	u8 lednum=0;
	u8 linenum=0;
	int8_t lednum_tmp = 0;
	int8_t i=0;

	for(i=0; i<sensorNum; i++) 		//��С�������������������������������
	{															//linenum������¼�ж������ߣ�line������¼�ڼ����ߡ�
		if((scaner->detail&(1<<i)))
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//�ȶ�ȡ��������������������⵽��1��Ϊ0��Ϊһ����
		}
	}
	scaner->lineNum = linenum;
	scaner->ledNum=lednum;
	
	if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)  //��ѭ��
	{
		for (i=0; i<sensorNum; i++)
		{
			lednum_tmp += (scaner->detail>>i)&0X01;
			_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			if ((scaner->detail>>i) & 0X01)
				if (!((scaner->detail>>(i+1))&0x01))
					break;
		}
		//printf("%f\r\n",_pow);
	}
	else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)  //��ѭ��
	{
		for (i=sensorNum-1; i>=0; i--)
		{
			lednum_tmp += (scaner->detail>>i)&0X01;
			_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			if ((scaner->detail>>i) & 0X01)
				if (!((scaner->detail>>(i-1))&0x01))
					break;
		}
		//printf("%f\r\n",_pow);
	}
	else//����Ѱ��
	{
		if (lednum <= 7)  ////��ֹ����ĵƴ����ĸ���  7
		{
			if (lednum >=4 )
				edge_ignore = 3;//ԭ����3

//			//******��ֱֹ�߳��·�߱�Ե�ߵ�Ӱ��
//			if(nodesr.nowNode.nodenum == N3 || nodesr.nowNode.nodenum == N6 || nodesr.nowNode.nodenum == N5)
//			{
//				edge_ignore = 5;
//			}
//			//**********
			
			for(i=edge_ignore; i<sensorNum-edge_ignore; i++) 
			{
				lednum_tmp += (scaner->detail>>i)&0X01;
				_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			}	
			//printf("%f\r\n",_pow);
		}
		else
			return;
	}
	
	if(lednum==0)
	{
		_pow=0;
	}	
	else
	{
		_pow/=(float)lednum_tmp;		//ȡƽ��
	}
	
	scaner->pow = _pow;
}




