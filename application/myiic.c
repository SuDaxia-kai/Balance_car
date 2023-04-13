#include "myiic.h"
#include "i2c.h"

/**********************************************************************************************************
*�� �� ��: i2c1_single_read
*����˵��: �жϷ�ʽ���е����Ĵ�����ȡ
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ ��ȡ����
*�� �� ֵ: ��ȡ״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}
/**********************************************************************************************************
*�� �� ��: i2c1_single_write
*����˵��: �жϷ�ʽ���е����Ĵ���д��
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ д������
*�� �� ֵ: д��״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
	if (HAL_I2C_Mem_Write(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*�� �� ��: i2c1_multi_read
*����˵��: �жϷ�ʽ���ж���Ĵ�����ȡ
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ ��ȡ���� ��ȡ����
*�� �� ֵ: ��ȡ״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt,9999) != HAL_OK)
		return -1;
	return 0;
}
