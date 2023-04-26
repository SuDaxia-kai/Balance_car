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
	float gyroz;
	float gyroy;
	float gyrox;

	float compensateZ;
	float compensatePitch;
};

extern void imu_wit_init(void);
extern volatile struct Imu imu;

#endif
