#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "vector3.h"

/*
�ϲ��ļ�:myiic.c,ֻ�����ʼ��mpu6050���ṩ��ȡ���ݵĺ������������κεĲ���
�²��ļ�:imu.c
*/


//�������ٶ�
#define GRAVITY_MSS 9.80665f
//�����ǿ̶�
#define GYRO_SCALE_2000  (0.0174532f / 16.4f)
#define GYRO_SCALE_1000  (0.0174532f / 32.8f)
#define GYRO_SCALE_500   (0.0174532f / 65.5f)
#define GYRO_SCALE_250   (0.0174532f / 131f)
//���ټƿ̶ȵ���
#define ACCEL_SCALE_16G   (GRAVITY_MSS / 2048.0f)
#define ACCEL_SCALE_8G    (GRAVITY_MSS / 4096.0f)
#define ACCEL_SCALE_4G    (GRAVITY_MSS / 8192.0f)
#define ACCEL_SCALE_2G    (GRAVITY_MSS / 16384.0f)

//���ټƳ߶�
#define ACCEL_MAX_1G      4096
#define ACCEL_SCALE ACCEL_SCALE_8G
//�����ǳ߶�
#define GYRO_CALIBRATION_COFF 0.0152672f

uint8_t MPU6050_Detect(void);
void MPU6050_Init(void);
void MPU6050_ReadAcc(Vector3i_t* acc);
void MPU6050_ReadGyro(Vector3i_t* gyro);
void MPU6050_ReadTemp(float* temp);

#endif
