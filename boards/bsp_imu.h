#ifndef BSP_IMU_H
#define BSP_IMU_H
#include "sys.h"
#include "string.h"

#define BUFFER_SIZE  200

struct Imu
{
	float yaw;
	float roll;
	float pitch;

	float compensateZ;
	float compensatePitch;
};

typedef union
{
	float info;
	char info_u[4];
} Robot_info;

extern char date_to_send[6];

extern void imu_receive_init(void);
extern volatile struct Imu imu;

#endif
