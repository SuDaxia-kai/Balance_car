#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "vector3.h"
#include "mpu6050.h"

//����Ƶ�� 100Hz
#define Sampling_Freq 200

//�������˲�������
extern Vector3i_t accDataFilter;
extern Vector3i_t gyroDataFilter;
extern Vector3i_t acceCorrectFilter;
extern Vector3i_t gyroCorrectFilter;
extern Vector3l_t Acc_Offset;
extern Vector3l_t Gyro_Offset;
extern float tempDataFilter;

void imu_init(void);
void get_imu_data(void);
void CalibrationAcc(void);
void CalibrationGyro(void);

#endif
