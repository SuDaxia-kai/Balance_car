#include "QR_action.h"
#include "bsp_QRscan.h"
#include "bsp_servo_iic.h"
#include "turn.h"
#include "bsp_led.h"

//ʹ��ͨ�� 0 -> ��   10   ֱ��  100  ����
//ʹ��ͨ�� 1 -> ����  0   �ٸ�  160 ����
//ʹ��ͨ�� 2 -> ����  160 �ٸ�  10  ����
//ʹ��ͨ�� 3 -> ͷ    0   160


void actions(uint8_t action)
{
	switch (action)
	{
		case 0://����ҡͷ
			//��һ��ҡͷ
			angle_write(3, 0);
			delay_ms(400);
			angle_write(3, 160);
			delay_ms(1000);
			
			//�ڶ���ҡͷ
			angle_write(3, 0);
			delay_ms(400);
			angle_write(3, 160);
			delay_ms(1000);
		
			break;
		
		case 1://����
			angle_write(1, 0);
			angle_write(2, 160);
			break;
		
		case 2://����
			angle_write(1, 160);
			angle_write(2, 0);
			break;
		
		case 3://�����֣�������
			//����
			angle_write(1, 0);
			delay_ms(200);//300
			angle_write(2, 160);
			delay_ms(200);
			//����
			angle_write(1, 160);
			delay_ms(200);
		  angle_write(2, 0);
			break;
		
//		case 4://�����֣�������
//			//����
//			angle_write(2, 160);
//			delay_ms(300);
//			angle_write(1, 10);
//			delay_ms(300);
//			//����
//			angle_write(2, 0);
//			delay_ms(300);
//		  angle_write(1, 160);
//			break;	
		
		case 5://ֱ��
			angle_write(0,10);
			break;
		
		case 6://�������
			angle_write(0,100);
			break;
		
	}	
}

//����ֱ��
void Stand (void)
{
	actions(5);
	actions(2);
}

//��������
void Down_Body(void)
{
	actions(6);
	actions(2);
}

//������׼������  ����ҡͷ
void PreAction(void)
{
	actions(0);
}

void Arrive (void)//ֱ�����㶯��
{
	actions(5);
	actions(3);
}

void Treasure(void)//�ҵ����ﶯ��
{
	actions(5);
	actions(1);
	//��ת360
	
	//actions(2);
}

void myAction(void)
{
	Shine_o_f();       //����һ��
	
	//���㶯��
	actions(5);  
	delay_ms(100);//800
	actions(3);
	delay_ms(200); 
	Turn_Angle_Relative(179);
	if(check_BW(nodesr.nowNode.nodenum)) //��ⱦ��
		{
			actions(1);
			delay_ms(100);//500
			Turn_Angle360();
			actions(2);
//			delay_ms(500);
		}
	actions(6);
//	delay_ms(100);
}

void usmart_set_servo(uint8_t choose, uint8_t angle)
{
	angle_write(choose,angle);
}

