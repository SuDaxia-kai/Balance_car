#include "mpu6050.h"
#include "myiic.h"
#include "i2c.h"

// The address of the MPU6050
#define MPU6050_ADR   0x68

//MPU6050寄存器地址
#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I	   	0x75
#define USER_CTRL		  0x6A
#define INT_PIN_CFG		0x37

/**********************************************************************************************************
*函 数 名: MPU6050_Detect
*功能说明: 检测MPU6050是否存在
*形    参: 无
*返 回 值: 1：存在，0：不存在
**********************************************************************************************************/
uint8_t MPU6050_Detect(void)
{
	uint8_t who_am_i;
	i2c1_single_read(MPU6050_ADR, WHO_AM_I, &who_am_i);
	if(who_am_i == MPU6050_ADR) return 1;
  else return 0;
}

/**********************************************************************************************************
*函 数 名: MPU6050_Init
*功能说明: MPU6050寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MPU6050_Init(void)
{
	int i = 0, j = 0;
	/* we do a  delay before initialization, 
	otherwise the data may be wrong when we power up the second time */
	for(i = 0; i < 1000; i++)
	{
		for(j = 0; j < 1000; j++)
		{
			;
		}
	}
	//关闭所有中断,解除休眠
	i2c1_single_write(MPU6050_ADR, PWR_MGMT_1, 0x00);
	//设置采样率
	i2c1_single_write(MPU6050_ADR, SMPLRT_DIV, 0x00);
	//内部低通滤波频率，加速度计94hz,陀螺仪98hz
	i2c1_single_write(MPU6050_ADR, MPU_CONFIG, 0x00);
	//设置陀螺仪的满量程500deg/s
	i2c1_single_write(MPU6050_ADR, GYRO_CONFIG, 0x08);
	//设置加速度计的满量程范围8g (4096 LSB/g)
	i2c1_single_write(MPU6050_ADR, ACCEL_CONFIG, 0x10);
}


/**********************************************************************************************************
*函 数 名: MPU6050_ReadAcc
*功能说明: MPU6050读取加速度传感器原始数据
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6050_ReadAcc(Vector3i_t* acc)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[6];

    retry = 5;
    while (retry--) {
        ret = i2c1_multi_read(MPU6050_ADR, ACCEL_XOUT_H, buffer, 6);
        if (ret == 0)
            break;
    }
	
    acc->x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    acc->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    acc->z = ((((int16_t)buffer[4]) << 8) | buffer[5]);

    //统一传感器坐标系（并非定义安装方向）
    acc->x = acc->x;
    acc->y = acc->y;
    acc->z = acc->z;
}

/**********************************************************************************************************
*函 数 名: MPU6050_ReadGyro
*功能说明: MPU6050读取陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6050_ReadGyro(Vector3i_t* gyro)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[6];

    retry = 5;
    while (retry--) {
        ret = i2c1_multi_read(MPU6050_ADR, GYRO_XOUT_H, buffer, 6);
        if (ret == 0)
            break;
    }

    gyro->x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    gyro->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    gyro->z = ((((int16_t)buffer[4]) << 8) | buffer[5]);

    //统一传感器坐标系（并非定义安装方向）
    gyro->x = gyro->x;
    gyro->y = gyro->y;
    gyro->z = gyro->z;
}

/**********************************************************************************************************
*函 数 名: MPU6050_ReadTemp
*功能说明: MPU6050读取温度传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6050_ReadTemp(float* temp)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[2];
    int16_t temperature_temp;

    retry = 5;
    while (retry--) {
        ret = i2c1_multi_read(MPU6050_ADR, TEMP_OUT_H, buffer, 2);
        if (ret == 0)
            break;
    }
    
    temperature_temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    *temp = 36.53f + (float)temperature_temp / 340.f;
}


