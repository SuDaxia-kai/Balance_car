#include "imu.h"
#include "mpu6050.h"
#include <string.h>
#include "stdio.h"

#include "Filter.h"
//#include <math.h>
//#include "ahrs.h"
//#include "time_cnt.h"
#include "stdio.h"

//#include "timers.h"
#include "usart.h"

//������˹�˲�����
static Butter_Parameter Gyro_Parameter;
static Butter_Parameter Accel_Parameter;
static Butter_Parameter Acce_Correct_Parameter;

//������˹�˲��ڲ�����
static Butter_BufferData Gyro_BufferData[3];
static Butter_BufferData Accel_BufferData[3];
static Butter_BufferData Butter_Buffer_Correct[3];

//������ԭʼ����
Vector3i_t accDataFilter;
Vector3i_t gyroDataFilter;

//��ͨ�˲��������
Vector3i_t acceCorrectFilter;
Vector3i_t gyroCorrectFilter;
float tempDataFilter;

//ƫ������
Vector3l_t Acc_Offset;
Vector3l_t Gyro_Offset;


/**********************************************************************************************************
*�� �� ��: imu_init
*����˵��: IMU��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void imu_init()
{
	//���ô������˲�����
	Set_Cutoff_Frequency(Sampling_Freq, 40,&Gyro_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 40,&Accel_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 3,&Acce_Correct_Parameter);
	//MPU6050��ʼ��
	MPU6050_Detect();
	MPU6050_Init();
}

/**********************************************************************************************************
*�� �� ��: get_imu_data
*����˵��: ��ȡIMU���ݲ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void get_imu_data()
{
	Vector3i_t accRawData;
	Vector3i_t gyroRawData;
	float tempRawData;
    
	//��ȡ���ٶȴ�����
	MPU6050_ReadAcc(&accRawData);
	//��ȡ�����Ǵ�����
	MPU6050_ReadGyro(&gyroRawData);
	//��ȡ�¶ȴ�����
	MPU6050_ReadTemp(&tempRawData);
	
	//�������Ư��У׼
	accRawData.x = accRawData.x - Acc_Offset.x ;
	accRawData.y = accRawData.y - Acc_Offset.y ;
	accRawData.z = accRawData.z - Acc_Offset.z ;
	
	gyroRawData.x = gyroRawData.x - Gyro_Offset.x;
	gyroRawData.y = gyroRawData.y - Gyro_Offset.y;
	gyroRawData.z = gyroRawData.z - Gyro_Offset.z;
	//printf("{ACC_z:%d,%d,%d}\n\r",accRawData.x,accRawData.y,accRawData.z);
//	printf("{Gyro:%d,%d,%d}\n\r",gyroRawData.x,gyroRawData.y,gyroRawData.z);
	
	//���������ݵ�ͨ�˲�
	gyroDataFilter.x = Butterworth_Filter(gyroRawData.x, &Gyro_BufferData[0], &Gyro_Parameter);
	gyroDataFilter.y = Butterworth_Filter(gyroRawData.y, &Gyro_BufferData[1], &Gyro_Parameter);
	gyroDataFilter.z = Butterworth_Filter(gyroRawData.z, &Gyro_BufferData[2], &Gyro_Parameter);
    
	//���ټ����ݵ�ͨ�˲�
	accDataFilter.x = Butterworth_Filter(accRawData.x, &Accel_BufferData[0], &Accel_Parameter);
	accDataFilter.y = Butterworth_Filter(accRawData.y, &Accel_BufferData[1], &Accel_Parameter);
	accDataFilter.z = Butterworth_Filter(accRawData.z, &Accel_BufferData[2], &Accel_Parameter);
	printf("{GYRO:%d,%d,%d}\n\r",gyroDataFilter.x,gyroDataFilter.y,gyroDataFilter.z);
	//printf("{ACC_z:%d,%d,%d}\n\r",accDataFilter.x,accDataFilter.y,accDataFilter.z);
	//�����ǽ������ݲ��˲�
  gyroCorrectFilter.x = gyroRawData.x;
  gyroCorrectFilter.y = gyroRawData.y;
  gyroCorrectFilter.z = gyroRawData.z;
	
	//���ټƽ������ݵ�ͨ�˲�
	acceCorrectFilter.x = Butterworth_Filter(accRawData.x, &Butter_Buffer_Correct[0], &Acce_Correct_Parameter);
	acceCorrectFilter.y = Butterworth_Filter(accRawData.y, &Butter_Buffer_Correct[1], &Acce_Correct_Parameter);
	acceCorrectFilter.z = Butterworth_Filter(accRawData.z, &Butter_Buffer_Correct[2], &Acce_Correct_Parameter);
	
	//printf("{ACC_z:%d,%d}\n\r",accDataFilter.z,acceCorrectFilter.z);
	//�¶����ݲ��˲�
	tempDataFilter = tempRawData;
}

/**********************************************************************************************************
*�� �� ��: vCalibrationAccTask
*����˵��: ����У׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void CalibrationAcc()
{
	int num_samples;
	Vector3l_t acce_sample_sum;
	Vector3i_t accRawData;
	
	//��ղ���
	acce_sample_sum.x = 0;
	acce_sample_sum.y = 0;
	acce_sample_sum.z = 0;
	
	printf("Acc Calibration Start\n\r");
	for(num_samples = 0 ; num_samples < 100 ; num_samples++){
		MPU6050_ReadAcc(&accRawData);//��ȡԭʼ����
		acce_sample_sum.x += accRawData.x;
		acce_sample_sum.y += accRawData.y;
		acce_sample_sum.z += (accRawData.z-4096);
//		printf("acce_sample_sum:%d,%d,%d\n\r",acce_sample_sum.x,acce_sample_sum.y,acce_sample_sum.z);
		HAL_Delay(5);
	}
	
	Acc_Offset.x = acce_sample_sum.x / num_samples;
	Acc_Offset.y = acce_sample_sum.y / num_samples;
	Acc_Offset.z = acce_sample_sum.z / num_samples;
	
//	printf("Acc_RawData_Offset:%d,%d,%d\n\r",Acc_Offset.x,Acc_Offset.y,Acc_Offset.z);
	printf("Acc Calibration Stop\n\r");
}

void CalibrationGyro()
{
	int num_samples;
	Vector3l_t gtro_sample_sum;
	Vector3i_t gyroRawData;
	
	//��ղ���
	gtro_sample_sum.x = 0;
	gtro_sample_sum.y = 0;
	gtro_sample_sum.z = 0;
	
	printf("Gyro Calibration Start\n\r");
	for(num_samples = 0 ; num_samples < 100 ; num_samples++){
		MPU6050_ReadGyro(&gyroRawData);//��ȡԭʼ����
		gtro_sample_sum.x += gyroRawData.x;
		gtro_sample_sum.y += gyroRawData.y;
		gtro_sample_sum.z += gyroRawData.z;
//		printf("acce_sample_sum:%d,%d,%d\n\r",gtro_sample_sum.x,gtro_sample_sum.y,gtro_sample_sum.z);
		HAL_Delay(5);
	}
	
	Gyro_Offset.x = gtro_sample_sum.x / num_samples;
	Gyro_Offset.y = gtro_sample_sum.y / num_samples;
	Gyro_Offset.z = gtro_sample_sum.z / num_samples;
	
//	printf("Gyro_RawData_Offset:%d,%d,%d\n\r",Gyro_Offset.x,Gyro_Offset.y,Gyro_Offset.z);
	printf("Gyro Calibration Stop\n\r");
}

