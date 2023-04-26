#include "bsp_imu.h"
#include "main.h"
#include "stdio.h"
#include "ahrs.h"

#define __WTGAHRS1__

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
  
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;

volatile struct Imu imu;

void imu_wit_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2,imu_rx_buf,BUFFER_SIZE);
}

void USART2_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		HAL_UART_DMAStop(&huart2); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   
		imu_rx_len = BUFFER_SIZE - temp;
		
		if(imu_rx_buf[0] == 0x55)
		{
			uint8_t sum = 0;
			
			#ifdef __WTGAHRS1__
			for (int i=11; i<21; i++)
				sum += imu_rx_buf[i];
			if (sum == imu_rx_buf[21])
			{
				if (imu_rx_buf[12] == 0x52)
				{
					imu.gyrox = 2000 * (short)((imu_rx_buf[14]<<8)|imu_rx_buf[13])/32768.0;
					imu.gyroy = 2000 * (short)((imu_rx_buf[16]<<8)|imu_rx_buf[15])/32768.0;
					imu.gyroz = 2000 * (short)((imu_rx_buf[18]<<8)|imu_rx_buf[17])/32768.0;
//					printf("{wit_angle_velocity:%f,%f,%f}\r\n",imu.gyrox,imu.gyroy,imu.gyroz);
				}
			}
			sum = 0;
			for (int i=22; i<32; i++)
				sum += imu_rx_buf[i];
			if(sum == imu_rx_buf[32])
			{
				if (imu_rx_buf[23] == 0x53)
				{
					//根据陀螺仪的安装方向确定roll和pitch
					imu.pitch = 180.0*(short)((imu_rx_buf[25]<<8)|imu_rx_buf[24])/32768.0;  
					imu.roll  = 180.0*(short)((imu_rx_buf[27]<<8)|imu_rx_buf[26])/32768.0;
					imu.yaw   = 180.0*(short)((imu_rx_buf[29]<<8)|imu_rx_buf[28])/32768.0;
					if(imu.yaw>360) imu.yaw -= 360;
					else if(imu.yaw<0) imu.yaw += 360;
				}
			}
			#endif
		}
		imu_rx_len = 0;
//		memset(imu_rx_buf,0,imu_rx_len);
	}
	HAL_UART_Receive_DMA(&huart2,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart2);
}



