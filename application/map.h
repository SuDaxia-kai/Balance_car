#ifndef __MAP_H
#define __MAP_H
#include "sys.h"

#include "map_message.h"

//#define FLEFT 	0x00	//��Ѱ��

#define NO      	(1<<0) 
#define DLEFT 		(1<<1)	//��ߺ���
#define DRIGHT 		(1<<2)	//�ұߺ���
#define CLEFT		  (1<<3)	//���б��
#define CRIGHT		(1<<4)	//�ұ�б��
#define MUL2SING	(1<<5)	//������һ��
#define MUL2MUL 	(1<<6)   //���������
#define AWHITE  	(1<<7)		//ȫ��

#define RESTPID	    (1<<8)
#define RESTMPUZ	(1<<9)		//������У׼
#define STOPTURN 	(1<<10)
#define SLOWDOWN	(1<<10)

#define LEFT_LINE      /* (1<<11)*/(1<<12)
#define RIGHT_LINE		/*(1<<12)*/(1<<13)
#define MCLEFT       (1<<13)
#define MCRIGHT      (1<<14)
//#define Thirdline		0x30

enum barriers {
	UpStage = 2,
	BBridge,
	BHill,
	LBHill,
	SM,
	View,
	View1,
	BACK,
	BSoutPole,
	QQB,
	BLBS,
	BLBL,
	DOOR,
	BHM,
	Scurve,
	IGNORE,
	UNDER,
};

extern u8 door1route[100];
extern u8 door2route[100];
extern u8 door3route[100];
extern u8 door4route[100];
extern u8 route[100];


enum MapNode {	//MapNode
	S1, P1, N1,B1,B2,B3,N2, P2, S2,
	P3, N3, N4, N5, N6, P4,
	N7,
	P5, B8, B9, N8, C1, C2,
	C3, N9, N10, N12, N13, P6,
	N14, S3, S4, N15, S5,
	C4, C5, B4, B5, B6, B7, 
	N16, N18, N19,
	P7, N20, N22, C6,
	C7, C8, C9, P8,
};

/**************************************/
//�����Ϣ
//flag 0λѰ�߷�ʽ��0��Ѱ�ߣ�1��Ѱ��
//flag 123λ����·�ڱ�־��	000����ߴ򵽣�001���ұߴ򵽣�010������ߣ�011�ұ����ߣ�100�����ɶ���һ��	
//flag 45λ��������Ŀ	
//flag 6λ��Ѱ�߷�ʽ�Ƿ�Ҫ�л���1��Ҫ�л���0����Ҫ�л�
//flag 7λ	��Ҫ������У��
//flag 8~11	
typedef struct _node{
	u8 nodenum;     //�������
	u16  flag;	    //����־λ
	float angle;	//�Ƕ�	
	u16	step;		//�߳�
	float speed;	//Ѱ���ٶ�
	u8 function;    //��㺯��
}NODE;

extern NODE Node[118];
/*************************/
//flag 0λ��1��������������0�������
//flag 1λ������·���ж�
//flag 2λ���Ƿ񵽴�·��
//flag 3λ��arrive��temp����
//flag 4λ��Z������
//flag 5λ��·�ߴ���λ ����
//flag 6λ��û����
typedef struct _nodesr{
	u8 flag;
	NODE nowNode;		//��ǰ���
	NODE nextNode;	//��һ���
}NODESR;

extern NODESR nodesr;

struct Map_State {
	u8 point;
	u8 routetime;
};
extern struct Map_State map;


u8 getNextConnectNode(u8 nownode,u8 nextnode);
void mapInit(void);
void Cross(void);
void map_function(u8 fun);
u8 deal_arrive(void);


#endif







