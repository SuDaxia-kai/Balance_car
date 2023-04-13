#include "myiic.h"
#include "i2c.h"

/**********************************************************************************************************
*函 数 名: i2c1_single_read
*功能说明: 中断方式进行单个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}
/**********************************************************************************************************
*函 数 名: i2c1_single_write
*功能说明: 中断方式进行单个寄存器写入
*形    参: 7bit从机地址 寄存器地址 写入数据
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
	if (HAL_I2C_Mem_Write(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c1_multi_read
*功能说明: 中断方式进行多个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt,9999) != HAL_OK)
		return -1;
	return 0;
}
