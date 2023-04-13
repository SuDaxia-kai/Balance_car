#ifndef __MYIIC_H
#define __MYIIC_H

#include <stdint.h>
#include "usart.h"

int i2c1_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data);
int i2c1_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data);
int i2c1_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);
#endif
