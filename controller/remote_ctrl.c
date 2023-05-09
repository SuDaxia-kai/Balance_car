#include "remote_ctrl.h"
#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "string.h"

struct PPM_Receiver ppm_receiver;
uint16_t capture_val = 0;
uint8_t tim_update_times = 0;
void ppm_receive_init()
{
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim5);
}

void TIM5_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	flag_idle = __HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC4);
	if((flag_idle != RESET))
	{ 
		__HAL_TIM_CLEAR_FLAG(&htim5, TIM_IT_CC4);
		
		static uint8_t ppm_ready = 0, ppm_sample_cnt = 0;
    static uint16_t ppm_buffer[10] = {0};
		capture_val = TIM5 -> CNT;
		TIM5 -> CNT = 0;
        
		//ppm接收状态
		if (1 == ppm_ready)
		{
			if (capture_val >= 900 && capture_val <= 2100)
			{
					ppm_buffer[ppm_sample_cnt++] = capture_val;
					if (ppm_sample_cnt >= 8)
					{
							ppm_ready = 0;
							memcpy(ppm_receiver.databuf, ppm_buffer, ppm_sample_cnt*sizeof(uint16_t));
							ppm_sample_cnt = 0;
					}
			}
			else

				ppm_ready = 0;
		}
		else
		{
				// 开启ppm接收状态条件1：捕获时间大于2.2ms，即为ppm帧间隔
				if (capture_val > 2200)
				{
						ppm_ready = 1;
						ppm_sample_cnt = 0;
				}
		}
        
			// 开启ppm接收状态条件2：已经产生定时器更新中断，间隔时长已经达到66.535ms
			if (1 == tim_update_times)
			{
					ppm_ready = 1;
					ppm_sample_cnt = 0;
					tim_update_times = 0;
			}
	}
	
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
			tim_update_times = 1;
    }
  }
	
}


