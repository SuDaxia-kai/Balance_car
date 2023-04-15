#ifndef _AHRSAUX_H
#define _AHRSAUX_H

#include <stdint.h>
#include "vector3.h"

extern float Yaw, Pitch, Roll;
extern Vector3f_t this_gyro;
extern float Sin_Pitch, Sin_Roll, Sin_Yaw;
extern float Cos_Pitch, Cos_Roll, Cos_Yaw;

void ahrs_init(void);
void ahrs_update(void);

#endif
