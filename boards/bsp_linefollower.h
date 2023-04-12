#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

#define LFB_SENSOR_NUM 16   //ѭ���崫��������
#define LFB_HALF_SENSOR_NUM 8

//��ǰ�ĺ���
#define Infrared_ahead  (uint8_t)!PBin(2)
#define Infrared_FrontLeft  (uint8_t)!PAin(12)

struct Line_Grays
{
	uint32_t Line;  //ѭ��ֵ
	float Value;  //��ֵ
	uint8_t LED_num;  //������
	int Line_width;  //����
	uint8_t Line_num;  //����
	float Line_Middle;  //ѭ������
};

struct Infrared_Sensor    //���⴫����
{
	uint8_t outside_left;
	uint8_t outside_right;
	uint8_t inside_left;
	uint8_t inside_right;
};

extern struct Line_Grays LFB_data;
extern volatile struct Infrared_Sensor infrared;

//���ڽ���ѭ�����ʼ��
void LFB_receive_init(void);

//���ַΪ1��ѭ���巢�Ͳ�ѯָ��
void LFB_send(void);

#endif

