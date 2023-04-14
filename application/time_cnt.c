#include "time_cnt.h"
#include "tim.h"
#include "tim_it.h"

//volatile uint32_t TIME_ISR_CNT;
//系统时间
Time_t Time_Sys;


/**********************************************************************************************************
*函 数 名: Get_Period
*功能说明: 获取时间周期
*形    参: 时间周期结构体
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Period(Testime *Time_Lab)
{
    uint32_t time_cnt;
    //关闭中断
    __disable_irq();
    time_cnt = TIM6->CNT;
	//如果还未初始化
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
        //如果在此期间产生中断并产生溢出
        if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) && time_cnt < 10000 / 2) {
            Time_Lab->Now_Time = 10000 * (TIME_ISR_CNT + 1) + time_cnt;
        } else {
            Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + time_cnt;
        }
		Time_Lab->Last_Time = Time_Lab->Now_Time;
		Time_Lab->Time_Delta = 0;
	} else {
        Time_Lab->Last_Time = Time_Lab->Now_Time;
        //单位us
        //如果在此期间产生中断并产生溢出
        if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) && time_cnt < 10000 / 2) {
            Time_Lab->Now_Time = 10000 * (TIME_ISR_CNT + 1) + time_cnt;
        } else {
            Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + time_cnt;
        }
        Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
    }
    //开启中断
    __enable_irq();
}
