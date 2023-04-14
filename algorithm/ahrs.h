#ifndef _AHRSAUX_H
#define _AHRSAUX_H

#include <stdint.h>

extern float Yaw, Pitch, Roll;
extern float Sin_Pitch, Sin_Roll, Sin_Yaw;
extern float Cos_Pitch, Cos_Roll, Cos_Yaw;

void ahrs_init(void);
void ahrs_update(void);

#endif
