#include "time_cnt.h"
#include "tim.h"
#include "tim_it.h"

//volatile uint32_t TIME_ISR_CNT;
//ϵͳʱ��
Time_t Time_Sys;


/**********************************************************************************************************
*�� �� ��: Get_Period
*����˵��: ��ȡʱ������
*��    ��: ʱ�����ڽṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_Time_Period(Testime *Time_Lab)
{
    uint32_t time_cnt;
    //�ر��ж�
    __disable_irq();
    time_cnt = TIM6->CNT;
	//�����δ��ʼ��
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
        //����ڴ��ڼ�����жϲ��������
        if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) && time_cnt < 10000 / 2) {
            Time_Lab->Now_Time = 10000 * (TIME_ISR_CNT + 1) + time_cnt;
        } else {
            Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + time_cnt;
        }
		Time_Lab->Last_Time = Time_Lab->Now_Time;
		Time_Lab->Time_Delta = 0;
	} else {
        Time_Lab->Last_Time = Time_Lab->Now_Time;
        //��λus
        //����ڴ��ڼ�����жϲ��������
        if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) && time_cnt < 10000 / 2) {
            Time_Lab->Now_Time = 10000 * (TIME_ISR_CNT + 1) + time_cnt;
        } else {
            Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + time_cnt;
        }
        Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
    }
    //�����ж�
    __enable_irq();
}
