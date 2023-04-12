#ifndef __RASPI_H
#define __RASPI_H
#include "sys.h"
#include "string.h"

#define D_SIZE  9  // 数据帧的长度
#define I_SIZE  6
#define BEGIN   0  
#define I_BEGIN 2
#define END     8

extern void raspi_receive_init(void);

#endif

