#include "bsp_imu.h"
#include "main.h"
#include "stdio.h"

#define __USE_ATK_IMU

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
char date_to_send[6];
  
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;



