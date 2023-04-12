#include "raspi.h"
#include "main.h"
#include "stdio.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static u8 robot_order[D_SIZE] = {0};
static u8 ras_state = 0;
uint8_t order_len = 0;
u8 counter = 0;
u8 infor[I_SIZE] = {0};

// 用DMA加快数据的传输
void raspi_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3, robot_order, D_SIZE);
}


void USART3_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);

		HAL_UART_DMAStop(&huart3); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   
		order_len = D_SIZE - temp; 
		
		if(ras_state==0 && robot_order[BEGIN]=='1')
		{
			ras_state = 1;
			counter++;
		}
		else if(ras_state==1 && robot_order[BEGIN+1]=='2')
		{
			ras_state = 2;
			counter++;
		}
		else if(ras_state==2)
		{
			// 此时已经可以接受信息帧了
			if(counter>=D_SIZE||robot_order[END] == '3')
			{
				ras_state = 3;
				/**
				 *@parameter
				 * velocity_A velocity_B velocity_C
				 * pitch yaw roll
				 * 6 params in total
				*/
				for(int iter = 0; iter < I_SIZE; iter++)
				{
					infor[iter] = robot_order[I_BEGIN + iter];
				}
				counter++;
			}
		}
		else if(ras_state == 3)
		{
			if(robot_order[END] == '3')
			{
				printf("Robot_information received is: %s \r\n", infor);
				ras_state = 0;
				counter = 0;
				memset(robot_order, 0, sizeof(robot_order));
			}
			else
			{
				ras_state = 0;
				counter = 0;
				memset(robot_order, 0, sizeof(robot_order));
			}
		}
	
	}
	HAL_UART_Receive_DMA(&huart3,robot_order,D_SIZE);
	HAL_UART_IRQHandler(&huart3);
}


